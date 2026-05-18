source /opt/ros/${ROS_DISTRO}/setup.bash  && \
    apt -y update && apt-get install -y \
    git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev ros-${ROS_DISTRO}-ament-cmake ros-${ROS_DISTRO}-nav2-map-server \
    && cd /root/ros2_ws/src && \
    if [ -d Stage ]; then echo "Stage already exists, skipping clone"; else git clone --branch ros2 https://github.com/tuw-robotics/Stage.git; fi && \
    if [ -d stage_ros2 ]; then echo "stage_ros2 already exists, skipping clone"; else git clone --branch humble https://github.com/tuw-robotics/stage_ros2.git; fi && \
    cd .. && \
    colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY --packages-select stage && \
    source install/setup.bash && \
    colcon build --symlink-install --packages-select stage_ros2 && \
    source install/setup.bash
