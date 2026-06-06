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
    python3-pip \
    python3-opencv \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2-py \
    ros-${ROS_DISTRO}-turtlebot3-simulations
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws
COPY ros2_ws/src/final_project_cv/requirements_ros.txt /tmp/final_project_cv_requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/final_project_cv_requirements.txt
RUN python3 -c "from ultralytics import FastSAM, YOLO; YOLO('yolo11n.pt'); FastSAM('FastSAM-s.pt')"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "defshell -bash" >> ~/.screenrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /root/ros2_ws/src
