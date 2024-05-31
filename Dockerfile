FROM osrf/ros:humble-desktop-full


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    terminator \
    && rm -rf /var/lib/apt/lists/*


# Example of copying a file
#COPY config/ /site_config/


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
  && apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3* \
  ros-humble-urdf-tutorial \
  ros-humble-tf2-tools \
  ros-humble-rmw-cyclonedds-cpp \
  && rm -rf /var/lib/apt/lists/*


USER ros


