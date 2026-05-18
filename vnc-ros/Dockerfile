FROM osrf/ros:humble-desktop

RUN apt-get -y update && apt-get install -y \
    curl

RUN apt-get -y update && apt-get install -y \
    iputils-ping \
    net-tools \
    wget \
    screen \
    git \
    nano \
    vim \
    htop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2-py \
    ros-${ROS_DISTRO}-turtlebot3-simulations
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "defshell -bash" >> ~/.screenrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /root/ros2_ws/src
