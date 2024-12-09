# Usa una imagen base oficial de ROS2 Melodic
FROM --platform=linux/amd64 osrf/ros:melodic-desktop

# Configura el entorno
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Configura el shell
SHELL ["/bin/bash", "-c"]

# Instala dependencias adicionales
RUN apt-get update && apt-get upgrade -yqq && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    python3-pip \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# Instala extensiones para ROS2
RUN pip3 install \
    empy \
    pytest-runner \
    setuptools \
    catkin_pkg \
    cryptography

# Fuente del setup de ROS2
RUN source /opt/ros/melodic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \ 
    cd ~/catkin_ws/src && \
    git clone -b ubuntu https://github.com/waveshare/jetbot_pro.git && \
    cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash

# Configura el entorno
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://192.168.111.234:11311" >> ~/.bashrc
