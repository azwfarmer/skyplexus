# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1* \
    ros-noetic-desktop \
    ros-noetic-cv-bridge \
    ros-noetic-velodyne \
    libcereal-dev \
    libyaml-cpp-dev \
    libspdlog-dev \
    git \
    ninja-build \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libsqlite3-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libc++-dev \
    libgflags-dev \
    libsuitesparse-dev \
    wget \
    clang \
    nano \
    less \
    libatlas-base-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libepoxy-dev \
    g++ \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    python3-dev \ 
    libopencv-dev \
    ros-noetic-vision-opencv \
    ros-noetic-image-pipeline \
    libopencv-calib3d-dev \
    libopencv-contrib-dev \
    libopencv-features2d-dev \
    libopencv-highgui-dev \
    libopencv-objdetect-dev \
    libopencv-stitching-dev \
    libopencv-videostab-dev \
    libdc1394-22-dev \
    && rm -rf /var/lib/apt/lists/*
# skipped catch2, --no-install-recommends libdc1394-dev libraw1394-dev libopenni-dev libeigen3-dev cmake libceres-dev libeigen3-dev cmake

WORKDIR /app
RUN wget https://github.com/Kitware/CMake/releases/download/v3.30.1/cmake-3.30.1.tar.gz
RUN tar xfvz cmake-3.30.1.tar.gz
WORKDIR /app/cmake-3.30.1
RUN ./bootstrap 
RUN make -j10
RUN make install

WORKDIR /app
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
RUN tar xfvz eigen-3.4.0.tar.gz
WORKDIR /app/eigen-3.4.0
RUN mkdir build
WORKDIR /app/eigen-3.4.0/build
RUN cmake ..
RUN make -j10
RUN make install

WORKDIR /app
# RUN wget https://github.com/gabime/spdlog/archive/refs/tags/v1.14.1.tar.gz -o spdlog.tar.gz
RUN git clone https://github.com/gabime/spdlog.git
# RUN tar xfvz spdlog.tar.gz
WORKDIR /app/spdlog
RUN git checkout v1.14.1
RUN mkdir build
WORKDIR /app/spdlog/build
RUN cmake ..
RUN make -j10
RUN make install


# http://ceres-solver.org/installation.html
WORKDIR /app
RUN wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz
RUN tar xfvz ceres-solver-2.2.0.tar.gz
WORKDIR /app/ceres-solver-2.2.0/
RUN mkdir ceres-bin
WORKDIR /app/ceres-solver-2.2.0/ceres-bin
RUN cmake ..
RUN make -j10
RUN make install

# # https://github.com/strasdat/Sophus
# # Needs CMake 3.24 or greater
WORKDIR /app
RUN git clone https://github.com/strasdat/Sophus.git
WORKDIR /app/Sophus
RUN git checkout 1.24.6
RUN mkdir build
WORKDIR /app/Sophus/build
RUN cmake .. -DSOPHUS_USE_BASIC_LOGGING=1
RUN make -j10
RUN make install

# # https://github.com/Neargye/magic_enum
WORKDIR /app
RUN git clone https://github.com/Neargye/magic_enum.git
WORKDIR /app/magic_enum/
RUN git checkout v0.9.6
RUN mkdir build
WORKDIR /app/magic_enum/build
RUN cmake ..
RUN make -j10
RUN make install

# https://github.com/stevenlovegrove/Pangolin
WORKDIR /app
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /app/Pangolin
RUN git checkout v0.9.2
RUN mkdir build
RUN cmake -B build -GNinja
RUN cmake --build build -j10
WORKDIR /app/Pangolin/build
RUN ninja install 
# # GIVEME THE PYTHON STUFF!!!! (Check the output to verify selected python version)
# cmake --build build f-t pypangolin_pip_install
# # Run me some tests! (Requires Catch2 which must be manually installed on Ubuntu.)
# cmake -B build -G Ninja -D BUILD_TESTS=ON
# cmake --build build
# cd build
# ctest


# #https://github.com/colmap/colmap
WORKDIR /app
RUN git clone https://github.com/colmap/colmap.git
WORKDIR /app/colmap
RUN git checkout  3.10
RUN mkdir build
WORKDIR /app/colmap/build
RUN sed -i '22d' ../cmake/FindDependencies.cmake
RUN cmake .. -GNinja
RUN ninja -j10
RUN ninja install


# # # Note, I modified colmap/cmake/FindDependencies.cmake to have the following before the CERES entry.
# # # find_package(GTest ${COLMAP_FIND_TYPE})
# # # find_package(absl ${COLMAP_FIND_TYPE})

# # # #     colmap -h
# # # # colmap gui

# # https://github.com/colmap/glomap?tab=readme-ov-file - requires Eigen 3.4 -DFETCH_COLMAP=0
# WORKDIR /app
# RUN git clone https://github.com/colmap/glomap.git
# WORKDIR /app/glomap
# RUN git checkout  1.0.0
# RUN mkdir build
# WORKDIR /app/glomap/build
# RUN cmake .. -GNinja 
# RUN ninja -j1
# RUN ninja install

# # # #glomap mapper --database_path DATABASE_PATH --output_path OUTPUT_PATH --image_path IMAGE_PATH

WORKDIR /app
COPY . /app

# # find_package(GTest REQUIRED) L5
# # find_package(absl REQUIRED) L6 glomap/cmake/FindDependencies.cmake

# https://github.com/Unsigned-Long/iKalibr/blob/master/docs/details/build_ikalibr.md
WORKDIR /app/src/ikalibr
RUN ./build_thirdparty.sh
WORKDIR /app
RUN . /opt/ros/noetic/setup.sh && catkin_make ikalibr_generate_messages
RUN . /opt/ros/noetic/setup.sh && catkin_make -j10
RUN . /opt/ros/noetic/setup.sh && catkin_make install

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# docker build -t myapp:v1 . --progress=plain
# docker run -it myapp:v1

# echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# source ~/.bashrc