# Docker

The image is built from the `Dockerfile` at the root of the repository. It has no ROS, builds natively on amd64 and arm64 (including Apple Silicon Macs), and includes the DaD + RoMa matcher for the automatic initial guess (with its weights, about 1.7 GB, so it runs offline) and the bag converter.

## Build image

```bash
git clone --recursive https://github.com/kolton-es/direct_visual_lidar_calibration
cd direct_visual_lidar_calibration
docker build -t direct_visual_lidar_calibration .
```

## Run programs

`docker_run.sh` mounts a data folder at `/data` and shows the viewers. They need OpenGL 3.3+, which the image renders on the CPU (Mesa llvmpipe), so no GPU is needed:

- `--vnc` (default on macOS): the viewers appear in the browser through noVNC (`http://localhost:6080/vnc.html`, opened by the script).
- `--x11` (default on Linux): X11 forwarding of `$DISPLAY`.
- `--no-display`: headless; a hidden virtual X server serves the viewers' hidden windows.

```bash
# Shell with the tools on PATH
./docker_run.sh --data /path/to/data

# Convert bags, preprocess, initial guess, calibrate, inspect
./docker_run.sh --data /path/to/data bag_to_dataset.py /data/ouster --out /data/ouster_dataset
./docker_run.sh --data /path/to/data preprocess -dv /data/ouster_dataset /data/ouster_preprocessed
./docker_run.sh --data /path/to/data initial_guess_manual /data/ouster_preprocessed
./docker_run.sh --data /path/to/data calibrate /data/ouster_preprocessed
./docker_run.sh --data /path/to/data viewer /data/ouster_preprocessed
```

`calibrate.sh` runs all of these steps in one command; see the README.
