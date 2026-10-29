#!/bin/bash
# Run the direct_visual_lidar_calibration image with a data folder mounted at
# /data and the viewers (preprocess -v, initial_guess_manual, calibrate,
# viewer) shown on the host.
#
#   ./docker_run.sh [--data DIR] [--image NAME] [--vnc | --x11 | --no-display]
#                   [--port N] [--screen WxH] [--no-open] [--dev]
#                   [--mount HOST_PATH:CONTAINER_PATH ...] [command ...]
#
# Display modes (the viewers need OpenGL 3.3+; the image renders it on the CPU
# with Mesa llvmpipe, so no GPU is needed):
#   --vnc         (default on macOS) the container runs its own X server and
#                 serves it to the browser with noVNC at
#                 http://localhost:6080/vnc.html, which the script opens. The
#                 page reconnects by itself, so one tab follows every run.
#                 Plain VNC clients can use localhost:5901.
#                 XQuartz cannot be used: its GLX stops at OpenGL 2.1.
#   --x11         (default on Linux) forward $DISPLAY with a copy of its xauth
#                 cookie; no `xhost +` needed.
#   --no-display  a hidden virtual X server; for headless runs
#                 (calibrate --background, preprocess without -v, ...).
# --port N serves noVNC on N (VNC on N-6080+5901) to run viewers side by side.
# --screen WxH sets the virtual screen size for --vnc (default 2560x1440).
# --no-open does not open the browser (e.g. when a noVNC tab is already open).
#
# --dev mounts this checkout's scripts over the installed ones, so Python
# edits take effect without rebuilding (C++ changes still need `docker build`).
# --mount adds a bind mount (repeatable).
#
# Examples:
#   ./docker_run.sh --data ~/calib_data                            # shell
#   ./docker_run.sh --data ~/calib_data viewer /data/preprocessed
#   ./docker_run.sh --data ~/calib_data --no-display calibrate /data/preprocessed --background
set -euo pipefail

IMAGE="direct_visual_lidar_calibration"
DATA_DIR="$(pwd)"
case "$(uname -s)" in
    Linux) MODE=x11 ;;
    *) MODE=vnc ;;
esac
PORT=6080
SCREEN=""
OPEN=1
DEV=0
MOUNTS=()
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while [ $# -gt 0 ]; do
    case "$1" in
        --data) DATA_DIR="$2"; shift 2 ;;
        --image) IMAGE="$2"; shift 2 ;;
        --vnc) MODE=vnc; shift ;;
        --x11) MODE=x11; shift ;;
        --no-display) MODE=none; shift ;;
        --port) PORT="$2"; shift 2 ;;
        --screen) SCREEN="$2"; shift 2 ;;
        --no-open) OPEN=0; shift ;;
        --dev) DEV=1; shift ;;
        --mount) MOUNTS+=(-v "$2"); shift 2 ;;
        --) shift; break ;;
        *) break ;;
    esac
done
DATA_DIR="$(cd "$DATA_DIR" && pwd)"

DOCKER_ARGS=(--rm --init -v "$DATA_DIR:/data")
if [ ${#MOUNTS[@]} -gt 0 ]; then
    DOCKER_ARGS+=("${MOUNTS[@]}")
fi
if [ "$DEV" = 1 ]; then
    for script in "$SCRIPT_DIR"/scripts/*.py; do
        DOCKER_ARGS+=(-v "$script:/usr/local/bin/$(basename "$script"):ro")
    done
fi
if [ -t 0 ] && [ -t 1 ]; then
    DOCKER_ARGS+=(-it)
fi

XAUTH_FILE="${TMPDIR:-/tmp}/direct_visual_lidar_calibration.docker.xauth"

setup_x11() {
    if [ -z "${DISPLAY:-}" ]; then
        echo "[docker_run] DISPLAY is not set; running without a display." >&2
        return 1
    fi
    rm -f "$XAUTH_FILE"; touch "$XAUTH_FILE"; chmod 600 "$XAUTH_FILE"
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge -
    DOCKER_ARGS+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw
                  -e XAUTHORITY=/tmp/.docker.xauth -v "$XAUTH_FILE:/tmp/.docker.xauth:ro")
}

setup_vnc() {
    local vnc_port=$((PORT - 6080 + 5901))
    URL="http://localhost:$PORT/vnc.html?autoconnect=1&reconnect=1&reconnect_delay=1000&resize=scale"
    DOCKER_ARGS+=(-e DVLC_VNC=1 -p "127.0.0.1:$PORT:6080" -p "127.0.0.1:$vnc_port:5900")
    if [ -n "$SCREEN" ]; then
        DOCKER_ARGS+=(-e "DVLC_SCREEN=$SCREEN")
    fi
    echo "[docker_run] Viewer display: $URL" >&2
    if [ "$OPEN" = 1 ]; then
        # Open the page once the container serves it.
        (
            for _ in $(seq 1 100); do
                curl -s -o /dev/null "http://localhost:$PORT/vnc.html" && break
                sleep 0.2
            done
            case "$(uname -s)" in
                Darwin) open "$URL" ;;
                *) command -v xdg-open >/dev/null && xdg-open "$URL" >/dev/null 2>&1 ;;
            esac
        ) >/dev/null 2>&1 &
    fi
}

case "$MODE" in
    x11) setup_x11 || MODE=none ;;
    vnc) setup_vnc ;;
esac

exec docker run "${DOCKER_ARGS[@]}" "$IMAGE" "$@"
