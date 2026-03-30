FROM --platform=linux/arm64 ghcr.io/emergent-swarm-solutions/jetson-ros-base:v1.4 AS base

# Use bash for RUN commands so ROS setup scripts can be sourced.
SHELL ["/bin/bash", "-c"]

# Default shell and ROS distribution.
ENV SHELL=/bin/bash
ENV ROS_DISTRO=humble

# Install rosdep with a fresh apt index.
# Clear apt lists before and after install to avoid stale metadata in cached layers.
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install -y python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# Initialise rosdep.
# Tolerate the case where the base image has already been initialised.
RUN rosdep init || true && \
    rosdep update

# Copy package manifest first so dependency resolution can be cached
# independently from source changes.
WORKDIR /ws/
COPY ./package.xml /ws/src/package/

# Install dependencies declared in package.xml.
# Refresh apt metadata immediately before rosdep install to avoid 404s.
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    rm -rf /var/lib/apt/lists/*

# Configure Fast RTPS defaults for ROS 2.
COPY docker/eprosima.xml /
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/eprosima.xml

# Copy package source into the workspace.
COPY . /ws/src/package

# Build only the target package.
# Use a single worker to reduce memory pressure and make compiler crashes easier to diagnose.
# console_direct+ gives clearer live logs during Docker builds.
# Keep symlink-install so config files under the install tree can be bind-mounted
# during development without rebuilding the image.
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build \
    --packages-select esstimator \
    --symlink-install \
    --executor sequential \
    --parallel-workers 1 \
    --event-handlers console_direct+ \
    --cmake-clean-cache \
    --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DBUILD_TESTING=OFF

# Install the entrypoint used by docker run / docker compose.
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
