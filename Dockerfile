# Base image with CUDA for Ubuntu 22.04
FROM nvidia/cuda:12.1.1-base-ubuntu22.04

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    nano \
    git \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    gnupg \
    software-properties-common \
    build-essential \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV key=value

# Add ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list 

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions 
    

# Install Gazebo Ignition
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list 
    

RUN apt-get update && apt-get install -y ignition-fortress && rm -rf /var/lib/apt/lists/*

# Install missing packages

RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-ros-gz-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz-sim \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup in every shell session
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Initialize rosdep
RUN rosdep init && rosdep update

# Set the default entrypoint to bash for flexibility
ENTRYPOINT ["bash"]
