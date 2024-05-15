FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update
RUN apt -y full-upgrade
RUN apt -y install cmake build-essential curl cppcheck g++ doxygen ruby-ronn libtinyxml2-dev software-properties-common
RUN apt -y install curl wget less nano lsb-release python3-pip gnupg git

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


RUN apt -y update
RUN apt -y install python3-vcstool python3-colcon-common-extensions

RUN mkdir -p /opt/gazebo/src
WORKDIR /opt/gazebo/src

RUN wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml -O - | vcs import

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt -y update

RUN sed -i 's/6.13//' gz-physics/.github/ci/packages.apt
RUN apt -y install $(sort -u $(find gz-launch -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
RUN apt -y install protobuf-compiler libprotobuf-dev libprotoc-dev libprotobuf-c-dev libtinyxml2-dev libfreeimage-dev libgts-dev swig pybind11-dev libeigen3-dev ruby-dev libavdevice-dev libswscale-dev libavformat-dev libavcodec-dev libavutil-dev libassimp-dev libgdal-dev libdart-all-dev libbullet-dev libbullet-extras-dev libdart-collision-bullet-dev

RUN apt -y install linux-headers-$(uname -r)
RUN apt-key del 7fa2af80
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb ; dpkg -i cuda-keyring_1.1-1_all.deb; rm -rf cuda-keyring_1.1-1_all.deb
RUN wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | tee /etc/apt/trusted.gpg.d/lunarg.asc
RUN wget -qO /etc/apt/sources.list.d/lunarg-vulkan-jammy.list http://packages.lunarg.com/vulkan/lunarg-vulkan-jammy.list

RUN apt -y update
RUN apt -y install cuda-toolkit libogre-next-2.3-dev libopengl-dev libegl-dev libogre-1.12-dev vulkan-sdk libjsoncpp-dev libzip-dev libzmq3-dev qtquickcontrols2-5-dev qtbase5-dev qt5-qmake qttools5-dev qttools5-dev-tools qtwebengine5-dev libqt5svg5-dev libqt5websockets5-dev python3-pip 

WORKDIR /opt/gazebo/
COPY WebsocketServer.* src/gz-launch/plugins/websocket_server/
RUN colcon graph
ENV MAKEFLAGS="-j 1"
RUN colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install --parallel-workers=1 --executor sequential

RUN mkdir -p /opt/ros-gz/src
WORKDIR /opt/ros-gz/src
RUN git clone https://github.com/gazebosim/ros_gz.git -b iron

WORKDIR /opt/ros-gz/
ENV GZ_VERSION=harmonic
RUN python3 -m pip install rosdep && python3 -m pip cache purge
RUN rosdep init && rosdep update
RUN rosdep install -r --from-paths src -i -y --rosdistro iron

COPY <<EOF ./build.sh
export GZ_VERSION=harmonic
export ROS_DISTRO=iron
. /opt/gazebo/install/setup.sh
. /opt/ros/iron/setup.sh
colcon build
EOF
RUN chmod +x build.sh
RUN ./build.sh

CMD /bin/bash

#Underwater sim added 

#CAPABILITIES FOR ARDUSUB
RUN apt install libgz-sim8-dev rapidjson-dev sudo -y  

# Install language
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
     tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user 'user', with the specified home directory and shell
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd -g $USER_GID user && \
    useradd -m -s /bin/bash -u $USER_UID -g $USER_GID user && \
    echo "user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN  usermod -a -G dialout user

# Switch to non-root user
USER user

WORKDIR /home/user

# Clone and set up ArduSub and related components
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git 
    
WORKDIR /home/user/ardupilot
#USER root
#RUN sudo usermod -a -G dialout user
#RUN echo "source /usr/share/bash-completion/completions/git" >> /home/user/.bashrc
#ENV USER=user
ARG DEBIAN_FRONTEND=noninteractive
ENV SKIP_AP_EXT_ENV=1 
ENV SKIP_AP_GRAPHIC_ENV=1 
ENV SKIP_AP_COV_ENV=1 
ENV SKIP_AP_GIT_CHECK=1
ENV USER=user
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y 
RUN ./waf configure --board sitl && \
    ./waf sub
WORKDIR /home/user/
# Clone and build ardupilot_gazebo
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
    mkdir -p ardupilot_gazebo/build && \
    cd ardupilot_gazebo/build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j4

USER root
RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/user/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> /home/user/.bashrc
RUN echo 'export GZ_SIM_RESOURCE_PATH=/home/user/ardupilot_gazebo/models:/home/user/ardupilot_gazebo/worlds:/home/user/colcon_ws/src/worlds/models/:/home/user/colcon_ws/src/worlds/:$GZ_SIM_RESOURCE_PATH' >> /home/user/.bashrc
# Cleanup to reduce image size
RUN apt update -y && apt install -y ros-dev-tools ros-iron-ros-base
RUN apt-get install -y ros-iron-ament-cmake-auto ros-iron-navigation2 ros-iron-xacro ros-iron-robot-localization ros-iron-joint-state-publisher ros-iron-ros2-control ros-iron-ros2-controllers
RUN apt-get update -y && apt-get install -y ros-iron-foxglove-msgs ros-iron-rosbridge-server ros-iron-rmw-cyclonedds-cpp ros-iron-cyclonedds cyclonedds-tools && apt-get -y clean
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo 'source /opt/ros/iron/setup.bash' >> /home/user/.bashrc
RUN echo 'source /opt/ros-gz/install/setup.bash' >> /home/user/.bashrc
RUN apt-get install -y ros-iron-joy ros-iron-rqt-graph

#INSTALL ORBSLAM3
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    ros-iron-rviz2 
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list || true

# bootstrap rosdep
ARG ROS_DISTRO=iron
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install deps
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-vision-opencv \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libglew-dev \
    libeigen3-dev \
    libboost-serialization-dev \
    libboost-python-dev \
    cmake \
    g++ \
    ninja-build \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libdc1394-dev \
    libraw1394-dev \
    libopenni-dev \
    python3-distutils && \
    rm -rf /var/lib/apt/lists/

# pangolin
WORKDIR /usr/src/pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build && \
    cmake --build build && \
    cd build && \
    make install
ENV PANGOLIN_DIR=/usr/src/pangolin/Pangolin

# build orbslam3
WORKDIR /home/user/ORB_SLAM3
COPY ./ORB_SLAM3 .
RUN ./build.sh
ENV ORB_SLAM3_DIR=/home/user/ORB_SLAM3

RUN echo "source /opt/ros/iron/setup.bash" >> /home/user/.bashrc
ENV ROS_DISTRO=$ROS_DISTRO
RUN echo 'export LD_LIBRARY_PATH=/home/user/ORB_SLAM3/lib:$LD_LIBRARY_PATH' >> /home/user/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/home/user/ORB_SLAM3/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH' >> /home/user/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/src/pangolin/Pangolin/build:$LD_LIBRARY_PATH' >> /home/user/.bashrc 

 #install ros2 bag
RUN apt-get update && apt-get install ros-iron-ros2bag \
                     ros-iron-rosbag2-storage-default-plugins -y
#install octomap ros packages
RUN apt install ros-iron-octomap-msgs -y
RUN apt install ros-iron-pcl-ros -y
RUN apt install ros-iron-octomap-ros -y


USER user 
# Set the final working directory
WORKDIR /home/user
#do this in the colcon_ws
#rosdep install -y --from-paths . --ignore-src --rosdistro iron --skip-keys="gz-transport13 gz-msgs10 gz-sim8 gz-math7"
