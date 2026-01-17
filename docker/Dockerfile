# Use the official ROS 2 Humble image as a base
FROM ros:humble

# Set the TURTLEBOT3 model environment variable
ENV TURTLEBOT3_MODEL=waffle

# Build-time arguments for SLAM Toolbox
ARG ROS_DISTRO=humble
ARG PREFIX=

# Install basic utilities and required dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    nano \
    git \
    build-essential \
    wget \
    python3-pip \
    gedit && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Remove conflicting source lists and keys
# RUN rm -f /etc/apt/sources.list.d/ros2.list ...
#     rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg ...

# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/ros-archive-keyring.gpg && \
#    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list && \
#    apt-get update

# Install ROS 2 packages, TurtleBot3, navigation, SLAM Toolbox, and additional packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-rviz-ogre-vendor \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-sim-demos \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-depthimage-to-laserscan \
    ros-$ROS_DISTRO-turtlebot3* \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-gripper-controllers \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget && \
    apt-get dist-upgrade -y && \
    rosdep update && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install colcon-argcomplete via pip
RUN pip install -U colcon-argcomplete && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Set up the environment for using Ogre libraries in rviz2
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/opt/rviz_ogre_vendor/lib' >> ~/.bashrc

# Set up the environment for ros_gz (Gazebo)
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'export GZ_VERSION=garden' >> ~/.bashrc

# Add RMW implementation
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Replace robot_model_type in waffle.yaml if it exists
RUN if [ -f /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml ]; then \
    sed -i 's/robot_model_type: "differential"/robot_model_type: "nav2_amcl::DifferentialMotionModel"/' /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml; \
    fi

# Export GAZEBO_MODEL_PATH
RUN echo 'export GAZEBO_MODEL_PATH=/workspace/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc

# Copy SLAM parameters into the container
COPY docker/slam_params /slam_params

# Record the version of SLAM Toolbox in the image
RUN if dpkg -s ros-$ROS_DISTRO-slam-toolbox >/dev/null 2>&1; then \
    dpkg -s ros-$ROS_DISTRO-slam-toolbox | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g' > /version.txt; \
    fi

# Install Ultralytics YOLO
RUN pip install ultralytics

# Install Ultralytics YOLO
RUN pip install numpy==1.24.4

# Set the working directory
WORKDIR /workspace/rosie

# Import dependencies
RUN apt-get update && \
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Copy your ROS2 workspace into the container (src folder contains packages)
# Note: This assumes docker build is run from repo root with: docker build -f docker/Dockerfile .

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source the setup scripts in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/rosie/install/setup.bash" >> ~/.bashrc

# Expose necessary ROS 2 ports
EXPOSE 11311 8080 8081

# Set the entrypoint to open a bash shell with the ROS 2 environment sourced
ENTRYPOINT ["/bin/bash", "-c", "source ~/.bashrc && exec /bin/bash"]

