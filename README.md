# direct_visual_lidar_calibration

This package provides a toolbox for LiDAR-camera calibration that is: 

- **Generalizable**: It can handle various LiDAR and camera projection models including spinning and non-repetitive scan LiDARs, and pinhole, fisheye, and omnidirectional projection cameras.
- **Target-less**: It does not require a calibration target but uses the environment structure and texture for calibration.
- **Single-shot**: At a minimum, only one pairing of a LiDAR point cloud and a camera image is required for calibration. Optionally, multiple LiDAR-camera data pairs can be used for improving the accuracy.
- **Automatic**: The calibration process is automatic and does not require an initial guess.
- **Accurate and robust**: It employs a pixel-level direct LiDAR-camera registration algorithm that is more robust and accurate compared to edge-based indirect LiDAR-camera registration.

Upstream documentation: [https://koide3.github.io/direct_visual_lidar_calibration/](https://koide3.github.io/direct_visual_lidar_calibration/)
(it describes the ROS version; the commands for this fork are below).

![213393920-501f754f-c19f-4bab-af82-76a70d2ec6c6](https://user-images.githubusercontent.com/31344317/213427328-ddf72a71-9aeb-42e8-86a5-9c2ae19890e3.jpg)

[Video](https://www.youtube.com/watch?v=7TM7wGthinc&feature=youtu.be)

## This fork: no ROS

This fork has no ROS dependency. It builds with plain CMake on Ubuntu 24.04,
natively on amd64 and arm64, including Docker on Apple Silicon Macs, and every
tool keeps its viewer.

Differences from upstream:
- `preprocess` reads a folder per sequence (an image, the camera intrinsics and
  one PCD/PLY file per LiDAR scan) instead of rosbags; see [Dataset format](#dataset-format).
  `--camera_info`, `--image` and `--points` name files in each sequence
  (replacing `--camera_info_topic`, `--image_topic`, `--points_topic`);
  inputs are detected automatically, so `-a` is optional.
- `bag_to_dataset.py` converts existing ROS1/ROS2 bags into that format with the
  pure-Python [rosbags](https://gitlab.com/ternaris/rosbags) library; no ROS
  installation is needed.
- Every other tool (`preprocess_map`, `initial_guess_manual`,
  `initial_guess_auto`, `calibrate`, `viewer`) is unchanged and reads/writes
  the same files as upstream.
- The automatic initial guess matches images with DaD keypoints and the RoMa
  dense matcher (`find_matches_roma.py`, MIT licensed) instead of SuperGlue
  (non-commercial license); it writes the same `*_matches.json` for
  `initial_guess_auto`.
- Added: `Dockerfile`, `docker_run.sh` and `calibrate.sh` (below). Removed:
  the ROS bag readers, `package.xml`, the ROS Dockerfiles and SuperGlue.

## Docker (preferred)

1. Clone with submodules and build the image (about 15 minutes the first time)

        git clone --recursive https://github.com/kolton-es/direct_visual_lidar_calibration
        cd direct_visual_lidar_calibration
        docker build -t direct_visual_lidar_calibration .

2. Run tools with `docker_run.sh`, which mounts your data folder at `/data`
   and shows the viewers on your screen. The tools are on `PATH`.

        ./docker_run.sh --data /path/to/data                       # interactive shell
        ./docker_run.sh --data /path/to/data viewer /data/preprocessed

   The viewers need OpenGL 3.3+. The image renders it on the CPU with Mesa
   (llvmpipe), so no GPU is needed, and shows the windows in one of these ways:
   - **`--vnc`** (default on macOS): the container runs its own X server with a
     window manager and serves it to your browser with noVNC; the script opens
     `http://localhost:6080/vnc.html`. Mouse, keyboard and right-click
     (two-finger click on a trackpad) go through. The page reconnects by
     itself, so one tab follows every run. XQuartz cannot be used here: its
     OpenGL stops at version 2.1. Other VNC clients can connect to
     `localhost:5901`. `--port N` serves another instance on port N,
     `--screen WxH` sets the virtual screen size (default 2560x1440) and
     `--no-open` skips opening the browser.
   - **`--x11`** (default on Linux): forwards `$DISPLAY` with a copy of its
     xauth cookie.
   - **`--no-display`**: headless. The viewers still need an X server for their
     hidden windows, so the container starts an invisible one.

   `--dev` mounts this checkout's Python scripts over the installed ones, so
   script edits take effect without rebuilding (C++ changes need
   `docker build`); `--mount HOST:CONTAINER` adds a bind mount (repeatable).

## Usage

### From recorded data, in one command

`calibrate.sh` runs every step in the Docker image: bag conversion (if
needed), preprocessing, the initial guess, fine registration, and prints the
result.

        ./calibrate.sh DATA_DIR [--bags PATH | --dataset DIR] [--init auto|manual|keep]
                       [--rotate_camera DEG] [--rotate_lidar DEG] [--show]
                       [--results DIR] [--force] [-- preprocess options]

`DATA_DIR` is mounted into the container; inputs and results live in it. With
`--bags PATH` the bags are converted to `DATA_DIR/<name of PATH>/` first
(reused on later runs); otherwise the dataset is `DATA_DIR` itself or the only
dataset folder in it. Results go to `DATA_DIR/<dataset>_results/`:
`calib.json` holds `T_lidar_camera`, and `logs/` the output of each step.
`--init manual` opens the point-picking GUI; `--show` also shows
preprocessing, the optimization and the result viewer. `--init keep` reruns
fine registration on an existing results folder with its initial guess.
Arguments after `--` go to `preprocess`; spinning LiDARs usually want
`-- --dynamic_lidar_integration`. See `./calibrate.sh --help`.

        # Ouster sample data from upstream (two ROS2 bags in data/ouster/)
        ./calibrate.sh data --bags ouster -- --dynamic_lidar_integration

The steps below are what it runs, for running them by hand.

### Dataset format

One directory per sequence (the counterpart of one rosbag). Each holds the
camera image, the camera intrinsics, and a folder of LiDAR scans named by their
timestamp in integer nanoseconds. Record the sensor at rest at the start of
each sequence: the image is matched with the first scans.

        dataset/
          sequence_a/
            image.png                  PNG, JPEG, BMP or TIFF (colour, grey or 16-bit)
            camera_info.json           optional with --camera_model ...
            points/1679988354559472320.pcd
            points/1679988354659472320.pcd
            ...
          sequence_b/
            ...

`camera_info.json`:

        {
          "distortion_model": "plumb_bob",
          "intrinsics": [fx, fy, cx, cy],
          "distortion_coeffs": [k1, k2, p1, p2, k3]
        }

`distortion_model` is `plumb_bob`, `fisheye`/`equidistant`, `omnidir` (add
`xi` to `intrinsics`) or `equirectangular`. Without the file, pass
`--camera_model`, `--camera_intrinsics` and `--camera_distortion_coeffs` to
`preprocess`.

Scans are PCD or PLY files with fields `x y z` (float or double), an intensity
field (`intensity` or `reflectivity` are found automatically, otherwise pass
`--intensity_channel NAME`) and, for spinning LiDARs, per-point times
(`t`, `time`, `time_stamp` or `timestamp`: uint32 nanoseconds or float/double
seconds relative to the scan, or absolute). Other fields are ignored.

### Convert bags (optional)

        bag_to_dataset.py BAG_OR_DIR [...] --out /data/dataset

Takes ROS1 `.bag` files, ROS2 bag directories (sqlite3 or mcap), or folders of
them. Each bag becomes a sequence named after it, holding the first image (raw
or compressed), the first camera info, and every point cloud as a binary PCD
with all of its fields. Topics are found by type; pass `--image_topic`,
`--camera_info_topic` or `--points_topic` when a bag has several. Duplicate
scans (same header stamp) are dropped; the ROS version skipped them as
timestamp rewinds.

### Preprocessing

        preprocess /data/dataset /data/preprocessed [-d] [-v]

`-d` (`--dynamic_lidar_integration`) builds the point cloud from a moving
spinning LiDAR (Ouster, Velodyne); without it the scans are accumulated as
static (Livox). `-v` shows the accumulation. Other options are in
`preprocess --help` and the [upstream documentation](https://koide3.github.io/direct_visual_lidar_calibration/programs/).
For a single image and a point cloud map, use `preprocess_map` as upstream
([example](docs/example_map.md)).

### Initial guess

Manual: right-click a 3D point in the viewer and the matching pixel in the
image window, press **Add picked points**, repeat (at least three, more is
better), press **Estimate**, check the overlay with `blend_weight`, and press
**Save**.

        initial_guess_manual /data/preprocessed

Automatic: `find_matches_roma.py` detects [DaD](https://github.com/Parskatt/dad)
keypoints in the camera and LiDAR intensity images and matches them through
the dense warp of [RoMa](https://github.com/Parskatt/RoMa) (outdoor weights,
DINOv2 ViT-L/14 backbone; the same models and weights as the `dad_roma`
matcher of image-matching-webui). The image includes the weights, so it runs
offline; on the CPU it takes about 20 s per sequence. Matching works best with
both images roughly upright; rotate them with `--rotate_camera`/`--rotate_lidar`
(90, 180 or 270, clockwise). `*_matches.png` shows the matches; see
`find_matches_roma.py --help` for the other options.

        find_matches_roma.py /data/preprocessed [--rotate_camera 90]
        initial_guess_auto /data/preprocessed

### Fine registration and result

        calibrate /data/preprocessed [--auto_quit | --background]
        viewer /data/preprocessed

`calib.json` then holds `results.T_lidar_camera = [x, y, z, qx, qy, qz, qw]`,
which maps camera-frame points to the LiDAR frame
(`p_lidar = T_lidar_camera * p_camera`). The manual initial guess takes
precedence over the automatic one when both are present.

## Local installation (Ubuntu 24.04)

Follow the `Dockerfile`: install the apt packages it lists, build GTSAM 4.2a9,
Ceres (commit `e47a42c`) and Iridescence v1.0.3 from source, optionally
install `rosbags` and RoMa/DaD ([installation](docs/installation.md)), then

        git clone --recursive https://github.com/kolton-es/direct_visual_lidar_calibration
        cmake -S direct_visual_lidar_calibration -B build -DCMAKE_BUILD_TYPE=Release
        cmake --build build -j$(nproc) && sudo cmake --install build

## Validation against the ROS version

On 2026-10-28 (Apple M4 Max, Docker arm64), upstream commit `02a0dc0` built for
ROS 2 Jazzy with the same GTSAM/Ceres/Iridescence/SuperGlue versions (SuperGlue
was the automatic initial guess of this fork then) was run on
upstream's Ouster sample (`ouster.tar.gz`, two ROS2 bags) and compared with this
fork on the converted dataset:

| Step | Result |
|---|---|
| Camera images, LiDAR intensity/index images | byte-identical |
| Accumulated point clouds (`.ply`) | same points; coordinates differ by at most 1 µm, which two runs of the ROS version also do |
| SuperGlue matches, `initial_guess_auto` | byte-identical / identical transform |
| `calibrate` on the same preprocessed files | identical `T_lidar_camera` |
| Whole pipeline, this fork vs the ROS version | 1.7 mm / 0.019° apart; two runs of the ROS version are 1.6 mm / 0.013° apart |

The whole pipeline (`calibrate.sh`, headless, automatic initial guess) took
54 s. The manual initial guess GUI was checked to open and render in the browser
display; picking points in it has not been exercised by an automated test.

On 2026-10-29 (same machine and data) the automatic initial guess was switched
from SuperGlue to DaD + RoMa and compared with the ROS version's final
`T_lidar_camera` from the run above:

| Initial guess | Inliers / matches | Initial guess error | Final `T_lidar_camera` error |
|---|---|---|---|
| SuperGlue (ROS version) | 4 / 519 | 108.9 mm / 1.20° | (reference) |
| DaD + RoMa (this fork) | 16 / 220 | 48.6 mm / 0.30° | 1.6 mm / 0.02° |

The final error is within the ROS version's own run-to-run spread. Matching
takes about 45 s for both sequences on the CPU in Docker (model loading
included) and runs with networking disabled; the whole pipeline took 79 s.

## Dependencies

- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [RoMa](https://github.com/Parskatt/RoMa) and [DaD](https://github.com/Parskatt/dad) [optional, automatic initial guess]
- [rosbags](https://gitlab.com/ternaris/rosbags) [optional, bag conversion]

## License

This package is released under the MIT license.

## Publication

Koide et al., General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox, ICRA2023, [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

## Contact

Kenji Koide, National Institute of Advanced Industrial Science and Technology (AIST), Japan
