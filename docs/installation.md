# Installation

!!!note
    The Docker image builds and runs everything without installation, including on Apple Silicon Macs: [Docker](docker.md). The `Dockerfile` is also the reference for the steps below (Ubuntu 24.04).

## Dependencies

- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [RoMa](https://github.com/Parskatt/RoMa) and [DaD](https://github.com/Parskatt/dad) [optional, automatic initial guess]
- [rosbags](https://gitlab.com/ternaris/rosbags) [optional, to convert ROS bags]

## Install Common dependencies

```bash
# Install dependencies
sudo apt install build-essential cmake git libeigen3-dev libomp-dev libboost-all-dev libtbb-dev libmetis-dev \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libpcl-dev libopencv-dev libfmt-dev \
  libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install GTSAM
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.2a9
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc)
sudo make install

# Install Ceres
git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver
cd ceres-solver
git checkout e47a42c2957951c9fafcca9995d9927e15557069
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
make -j$(nproc)
sudo make install

# Install Iridescence for visualization
git clone https://github.com/koide3/iridescence --recursive
cd iridescence && git checkout v1.0.3 && git submodule update --init --recursive
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

## Install RoMa and DaD (Optional)

Needed for the automatic initial guess (`find_matches_roma.py`). These are the versions the `Dockerfile` pins; their own dependency lists include training-only packages, so they are installed without them. The weights (about 1.7 GB) download to the torch hub cache on first use.

```bash
pip3 install numpy opencv-python matplotlib torch torchvision kornia einops loguru
pip3 install --no-deps "romatch @ git+https://github.com/Parskatt/RoMa.git@77f8d68803526dcddfd9b7a46bc76125bdc25f15" \
                       "dad @ git+https://github.com/Parskatt/dad.git@90103c8093ff7121acaaddec1b3a07822ce41db1"
```

## Install rosbags (Optional)

Only needed to convert existing ROS bags to dataset folders (`bag_to_dataset.py`); it is a pure-Python reader, not ROS.

```bash
pip3 install rosbags
```

## Build direct_visual_lidar_calibration

```bash
git clone https://github.com/kolton-es/direct_visual_lidar_calibration.git --recursive
cmake -S direct_visual_lidar_calibration -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build   # tools go to /usr/local/bin
```
