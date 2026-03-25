FROM --platform=linux/arm64 ghcr.io/emergent-swarm-solutions/jetson-ros-base:v1.4 as base
# Set shell to bash (instead of "sh") for Docker RUN.
SHELL ["/bin/bash", "-c"]
# Set shell to bash (instead of "sh") when opening a new terminal.
ENV SHELL=/bin/bash
ENV ROS_DISTRO=humble

# Install python3-rosdep to install the dependencies defined in package.xml
RUN apt-get update && apt-get install python3-rosdep -y
RUN rosdep init
RUN rosdep update

# Install the package.xml dependencies
WORKDIR /ws/
COPY ./package.xml /ws/src/package/
RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Defines standard Fast RTPS configuration for ROS2
COPY docker/eprosima.xml /
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/eprosima.xml

# Copy and build this package
COPY . /ws/src/package
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    colcon build \
    --symlink-install \
    --parallel-workers $(nproc) \
    --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_BUILD_TYPE=Release

# The entrypoint always runs before the commands we specify in the docker-compose.yml
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh 
ENTRYPOINT [ "/entrypoint.sh" ]