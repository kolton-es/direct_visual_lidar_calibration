#!/bin/bash
# LiDAR-camera calibration end to end (everything after the recording), using
# the direct_visual_lidar_calibration Docker image.
#
#   ./calibrate.sh DATA_DIR [options] [-- extra preprocess args]
#
# DATA_DIR holds the input, either a dataset folder (one directory per
# sequence with image.png, camera_info.json and points/<ns>.pcd; see README)
# or ROS bags to convert to one. The script writes:
#   DATA_DIR/NAME/                 dataset (converted from bags with --bags)
#   DATA_DIR/NAME_results/         preprocessed data and the calibration
#     calib.json                   results.T_lidar_camera = [x y z qx qy qz qw]
#     <sequence>.png/.ply, <sequence>_lidar_*.png, *_matches.png, logs/
#
# Input (one of):
#   --dataset DIR   dataset folder (default: DATA_DIR itself if it holds
#                   sequences, else the only such folder in DATA_DIR)
#   --bags PATH     ROS1 .bag files / ROS2 bag directories (or a folder of
#                   them) to convert first, without ROS; the dataset goes to
#                   DATA_DIR/<basename of PATH> (reused if already there)
#
# Options:
#   --init MODE     initial guess: auto (DaD + RoMa matching + RANSAC, default), manual
#                   (pick 2D-3D point pairs in the GUI), or keep (use the
#                   initial guess already in the results folder)
#   --rotate_camera DEG / --rotate_lidar DEG
#                   rotate images for matching (90, 180 or 270; CW)
#   --show          show the viewers: points accumulation, calibration
#                   progress and the result viewer (manual init always shows)
#   --results DIR   results folder (default: DATA_DIR/NAME_results; must be in DATA_DIR)
#   --force         replace an existing results folder (not with --init keep)
#   --image NAME    Docker image (default: direct_visual_lidar_calibration)
#
# Everything after `--` is passed to preprocess, e.g.
#   ./calibrate.sh data --bags ouster -- --dynamic_lidar_integration
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="$SCRIPT_DIR/docker_run.sh"

usage() { sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | grep '^#' | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
die() { echo "calibrate.sh: $*" >&2; exit 1; }

DATA_DIR=""
DATASET=""
BAGS=""
INIT="auto"
ROTATE_CAMERA=0
ROTATE_LIDAR=0
SHOW=0
FORCE=0
RESULTS=""
IMAGE="direct_visual_lidar_calibration"
EXTRA=()

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) usage 0 ;;
        --dataset) DATASET="$2"; shift 2 ;;
        --bags) BAGS="$2"; shift 2 ;;
        --init) INIT="$2"; shift 2 ;;
        --rotate_camera) ROTATE_CAMERA="$2"; shift 2 ;;
        --rotate_lidar) ROTATE_LIDAR="$2"; shift 2 ;;
        --show) SHOW=1; shift ;;
        --force) FORCE=1; shift ;;
        --results) RESULTS="$2"; shift 2 ;;
        --image) IMAGE="$2"; shift 2 ;;
        --) shift; EXTRA=("$@"); break ;;
        -*) die "unknown option $1 (see --help)" ;;
        *) [ -z "$DATA_DIR" ] || die "only one DATA_DIR allowed"; DATA_DIR="$1"; shift ;;
    esac
done
[ -n "$DATA_DIR" ] || usage 1
[ -d "$DATA_DIR" ] || die "no such directory: $DATA_DIR"
DATA_DIR="$(cd "$DATA_DIR" && pwd)"
case "$INIT" in auto|manual|keep) ;; *) die "--init must be auto, manual or keep" ;; esac
[ -z "$DATASET" ] || [ -z "$BAGS" ] || die "--dataset and --bags are exclusive"

# Path of a file inside DATA_DIR as the container sees it (/data/...).
container_path() {
    local abs
    abs="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
    case "$abs" in
        "$DATA_DIR") echo "/data" ;;
        "$DATA_DIR"/*) echo "/data/${abs#"$DATA_DIR"/}" ;;
        *) die "$1 must be inside $DATA_DIR (it is mounted into the container)" ;;
    esac
}
in_data_dir() { case "$1" in /*) echo "$1" ;; *) [ -e "$1" ] && echo "$(pwd)/$1" || echo "$DATA_DIR/$1" ;; esac; }

# A sequence is a directory with a subdirectory of PCD/PLY scans.
is_sequence() {
    local sub
    for sub in "$1"/*/; do
        ls "$sub" 2>/dev/null | grep -qiE '^[0-9]+\.(pcd|ply)$' && return 0
    done
    return 1
}
is_dataset() {
    local seq
    for seq in "$1"/*/; do is_sequence "$seq" && return 0; done
    return 1
}

docker info >/dev/null 2>&1 || die "Docker is not responding; start Docker Desktop and retry"
# `docker image inspect` does not default to :latest on every Docker version
[ -n "$(docker images -q "$IMAGE" 2>/dev/null)" ] \
    || die "Docker image '$IMAGE' not found; build it with: docker build -t $IMAGE $SCRIPT_DIR"

# --- inputs ------------------------------------------------------------------
if [ -n "$BAGS" ]; then
    BAGS="$(in_data_dir "$BAGS")"
    [ -e "$BAGS" ] || die "no such bag path: $BAGS"
    DATASET="$DATA_DIR/$(basename "${BAGS%.bag}")"
    [ "$DATASET" != "$BAGS" ] || DATASET="${DATASET}_dataset"
elif [ -z "$DATASET" ]; then
    if is_dataset "$DATA_DIR"; then
        DATASET="$DATA_DIR"
    else
        candidates=()
        for dir in "$DATA_DIR"/*/; do
            is_dataset "$dir" && candidates+=("${dir%/}")
        done
        [ ${#candidates[@]} = 1 ] || die "found ${#candidates[@]} dataset folders in $DATA_DIR; pass --dataset or --bags"
        DATASET="${candidates[0]}"
    fi
else
    DATASET="$(in_data_dir "$DATASET")"
fi
DATASET="${DATASET%/}"
NAME="$(basename "$DATASET")"
[ "$DATASET" != "$DATA_DIR" ] || NAME="dataset"

if [ -z "$RESULTS" ]; then
    RESULTS="$DATA_DIR/${NAME}_results"
else
    RESULTS="$(in_data_dir "$RESULTS")"
fi
C_RESULTS="$(container_path "$RESULTS")"
LOGS="$RESULTS/logs"

if [ "$INIT" = keep ]; then
    [ -f "$RESULTS/calib.json" ] || die "--init keep needs an existing $RESULTS/calib.json"
    [ "$FORCE" = 0 ] || die "--force would delete the initial guess that --init keep reuses"
elif [ -e "$RESULTS" ]; then
    [ "$FORCE" = 1 ] || die "$RESULTS exists; pass --force to replace it (or --init keep to recalibrate it)"
    rm -rf "$RESULTS"
fi
mkdir -p "$LOGS"

RUN_ARGS=(--data "$DATA_DIR" --image "$IMAGE")
if [ "$SHOW" = 0 ] && [ "$INIT" != manual ]; then
    RUN_ARGS+=(--no-display)
fi

# run LOG_NAME COMMAND...: run a step in the container, tee its output to logs/.
step=0
run() {
    local log="$LOGS/$1.log"; shift
    step=$((step + 1))
    echo "=== [$step] $*"
    if ! "$RUNNER" "${RUN_ARGS[@]}" "$@" 2>&1 | tee "$log"; then
        die "step failed: $* (log: $log)"
    fi
}

# --- 1. bags -> dataset --------------------------------------------------------
if [ -n "$BAGS" ]; then
    if is_dataset "$DATASET"; then
        echo "Reusing the dataset converted earlier: $DATASET"
    else
        run bag_to_dataset bag_to_dataset.py "$(container_path "$BAGS")" --out "$(container_path "$DATASET")"
    fi
fi
is_dataset "$DATASET" || die "$DATASET holds no sequences (directories with points/<ns>.pcd)"
C_DATASET="$(container_path "$DATASET")"

# --- 2. preprocess ---------------------------------------------------------------
if [ "$INIT" != keep ]; then
    pre=(preprocess "$C_DATASET" "$C_RESULTS")
    [ "$SHOW" = 0 ] || pre+=(--visualize)
    run preprocess "${pre[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
fi

# --- 3. initial guess --------------------------------------------------------------
case "$INIT" in
    auto)
        run find_matches find_matches_roma.py "$C_RESULTS" \
            --rotate_camera "$ROTATE_CAMERA" --rotate_lidar "$ROTATE_LIDAR"
        run initial_guess_auto initial_guess_auto "$C_RESULTS"
        ;;
    manual)
        echo "Pick at least three 2D-3D point pairs, press Estimate, then Save and close the window."
        run initial_guess_manual initial_guess_manual "$C_RESULTS"
        ;;
esac
grep -qE '"init_T_lidar_camera(_auto)?"' "$RESULTS/calib.json" \
    || die "no initial guess in $RESULTS/calib.json (manual: press Save before closing)"

# --- 4. fine registration -----------------------------------------------------------
if [ "$SHOW" = 1 ]; then
    run calibrate calibrate "$C_RESULTS" --auto_quit
else
    run calibrate calibrate "$C_RESULTS" --background
fi

# --- 5. result -----------------------------------------------------------------------
echo
echo "Calibration: $RESULTS/calib.json"
python3 - "$RESULTS/calib.json" <<'EOF' || true
import json, sys
results = json.load(open(sys.argv[1]))["results"]
for key in ("init_T_lidar_camera", "init_T_lidar_camera_auto", "T_lidar_camera"):
    if key in results:
        print("  %-26s [x y z qx qy qz qw] = %s" % (key, " ".join("%.6f" % v for v in results[key])))
EOF
if [ "$SHOW" = 1 ]; then
    run viewer viewer "$C_RESULTS"
else
    echo "Inspect it with: $RUNNER --data $DATA_DIR viewer $C_RESULTS"
fi
