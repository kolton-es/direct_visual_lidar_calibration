FROM ubuntu:24.04

# Plain Ubuntu 24.04, no ROS. Builds natively on amd64 and arm64 (e.g. Apple
# Silicon). GTSAM, Ceres and Iridescence are built from pinned sources; the
# viewers render with Mesa (llvmpipe), so they need no GPU.
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates wget \
        libeigen3-dev libboost-all-dev libtbb-dev libmetis-dev libomp-dev \
        libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
        libpcl-dev libopencv-dev libfmt-dev \
        libglm-dev libglfw3-dev libpng-dev libjpeg-dev \
        libgl1-mesa-dri libglx-mesa0 mesa-utils \
        python3-pip python3-numpy python3-opencv python3-matplotlib \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/src
RUN git clone https://github.com/borglab/gtsam.git && cd gtsam && git checkout 4.2a9 \
 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=ON \
 && cmake --build build -j$(nproc) && cmake --install build && cd .. && rm -rf gtsam

RUN git clone https://github.com/ceres-solver/ceres-solver && cd ceres-solver \
 && git checkout e47a42c2957951c9fafcca9995d9927e15557069 \
 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF \
 && cmake --build build -j$(nproc) && cmake --install build && cd .. && rm -rf ceres-solver

RUN git clone https://github.com/koide3/iridescence.git && cd iridescence && git checkout v1.0.3 \
 && git submodule update --init --recursive \
 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build -j$(nproc) && cmake --install build && cd .. && rm -rf iridescence
ENV LD_LIBRARY_PATH=/usr/local/lib

# DaD keypoints + RoMa dense matching (automatic initial guess,
# find_matches_roma.py) on CPU PyTorch, and rosbags (a pure-Python bag reader)
# for bag_to_dataset.py. RoMa and DaD are installed without their dependency
# lists, which pull in training-only packages (wandb, albumentations, poselib);
# inference needs only the packages listed here.
RUN pip3 install --no-cache-dir --break-system-packages \
        torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cpu \
 && pip3 install --no-cache-dir --break-system-packages \
        rosbags==0.10.9 kornia==0.9.0 einops==0.8.2 loguru==0.7.3 \
 && pip3 install --no-cache-dir --break-system-packages --no-deps \
        "romatch @ git+https://github.com/Parskatt/RoMa.git@77f8d68803526dcddfd9b7a46bc76125bdc25f15" \
        "dad @ git+https://github.com/Parskatt/dad.git@90103c8093ff7121acaaddec1b3a07822ce41db1"
# Weights in the torch hub cache, where RoMa and DaD look before downloading, so
# matching works offline: RoMa outdoor, its DINOv2 ViT-L/14 backbone and DaD
# v0.1.0 (the same files as image-matching-webui's dad_roma).
ENV TORCH_HOME=/opt/torch
RUN mkdir -p $TORCH_HOME/hub/checkpoints && cd $TORCH_HOME/hub/checkpoints \
 && wget -q https://github.com/Parskatt/storage/releases/download/roma/roma_outdoor.pth \
 && wget -q https://dl.fbaipublicfiles.com/dinov2/dinov2_vitl14/dinov2_vitl14_pretrain.pth \
 && wget -q https://github.com/Parskatt/dad/releases/download/v0.1.0/dad.pth \
 && printf '%s\n' \
        "c7a45c80d41ad788a63c641d1b686d7cb3f297f40097c6f4e75039889e5cc8ba  roma_outdoor.pth" \
        "d5383ea8f4877b2472eb973e0fd72d557c7da5d3611bd527ceeb1d7162cbf428  dinov2_vitl14_pretrain.pth" \
        "7d827844649faf5213ba97e7adb43e16b936dd3d11ab2d0fc0d602c67fae781f  dad.pth" \
    | sha256sum -c -

# The viewers need an X server even when hidden (calibrate --background), so
# the entrypoint starts a virtual one (Xvfb) when no DISPLAY is passed in, and
# can show it in a browser through noVNC (docker/entrypoint.sh).
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb xauth x11vnc novnc websockify openbox \
 && rm -rf /var/lib/apt/lists/*
COPY docker/entrypoint.sh /entrypoint.sh
# Mesa's software rasterizer: OpenGL 4.5 with no GPU.
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

COPY . /opt/src/direct_visual_lidar_calibration
RUN cmake -S /opt/src/direct_visual_lidar_calibration -B /opt/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /opt/build -j$(nproc) && cmake --install /opt/build && ldconfig

WORKDIR /data
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
