FROM ros:noetic-robot-focal

ENV CERES_VERSION="1.12.0"
ENV GTSAM_VERSION="4.2.0"
ARG OPENCV_VERSION="4.2.0"
ENV CATKIN_WS=/root/catkin_ws

      # set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      libboost-all-dev \
      git \
      wget \
      unzip \
      python3-rosdep \
      python3-rosinstall \
      python3-rosinstall-generator \
      python3-wstool \
      build-essential\
      python3-rosdep

RUN   rosdep update

RUN   git clone https://ceres-solver.googlesource.com/ceres-solver && \
      cd ceres-solver && \
      git checkout tags/${CERES_VERSION} && \
      mkdir build && cd build && \
      cmake .. && \
      make -j$(USE_PROC) install && \
      rm -rf ../../ceres-solver && \
      mkdir -p $CATKIN_WS/src/VINS-Fusion/

# RUN   git clone https://github.com/borglab/gtsam.git && \
#       cd gtsam && \
#       git checkout tags/${GTSAM_VERSION} && \
#       mkdir build && cd build && \
#       cmake .. && \
#       make -j$(USE_PROC) install


RUN   wget https://github.com/borglab/gtsam/archive/refs/tags/4.2.tar.gz && \
      tar -xvzf 4.2.tar.gz && \
      cd gtsam-4.2 && \
      mkdir build && cd build && \
      cmake .. && \
      make -j$(USE_PROC) install

#install opencv 4.6.0
RUN   apt update && \
      apt install libgtk2.0-dev -y && \
      wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -O opencv.zip && \
      unzip opencv.zip && \
      rm opencv.zip && \
      git clone https://github.com/opencv/opencv_contrib.git -b ${OPENCV_VERSION}
RUN   cd opencv-${OPENCV_VERSION} && \
      mkdir build && cd build && \
      cmake .. \
            -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D WITH_CUDA=OFF \
            -D WITH_CUDNN=OFF \
            -D WITH_CUBLAS=OFF \
            -D WITH_TBB=OFF \
            -D OPENCV_DNN_CUDA=OFF \
            -D OPENCV_ENABLE_NONFREE=ON \
            -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
            -D BUILD_EXAMPLES=OFF \
            -D BUILD_opencv_java=OFF \
            -D BUILD_opencv_python=OFF \
            -D BUILD_TESTS=OFF \
            -D BUILD_PERF_TESTS=OFF \
            -D BUILD_opencv_apps=OFF \
            -D BUILD_LIST=calib3d,features2d,highgui,dnn,imgproc,imgcodecs,\
cudev,cudaoptflow,cudaimgproc,cudalegacy,cudaarithm,cudacodec,cudastereo,\
cudafeatures2d,xfeatures2d,tracking,stereo,\
aruco,videoio,ccalib && \
      make -j$(nproc) && \
      make install 

RUN   sh \
      -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
      > /etc/apt/sources.list.d/ros-latest.list' && \
      wget http://packages.ros.org/ros.key -O - | sudo apt-key add - && \
      apt-get update && \
      apt-get install -y python3-catkin-tools

RUN   apt-get install -y ros-noetic-cv-bridge \
      ros-noetic-image-transport

# Copy VINS-Fusion
COPY ./ $CATKIN_WS/src/VINS-Fusion/
# use the following line if you only have this dockerfile
# RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git

# Build VINS-Fusion

WORKDIR $CATKIN_WS
ENV TERM=xterm
ENV PYTHONIOENCODING=UTF-8
RUN   catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
      catkin build -j$(USE_PROC) && \
      sed -i '/exec "$@"/i \
                  source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh