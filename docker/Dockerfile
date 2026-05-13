# Mixer ROS 2 bridge runtime image.
#
# Builds the mixer_comm + mixer_comm_msgs packages on top of the official
# ros:jazzy-ros-base image (multi-arch: amd64 desktop + arm64 Jetson).
# The Mixer firmware tree is intentionally NOT included -- dongles are
# expected to be flashed before being plugged into the Jetson.

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Copy only the ROS 2 workspace; .dockerignore strips Mixer/, build/, etc.
COPY ros2_ws/src /ws/src
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+

# Source ROS + workspace overlays for every shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
    && echo 'source /ws/install/setup.bash' >> /etc/bash.bashrc

# Default to local-only DDS so two Jetsons don't auto-discover each other.
# Override at `docker run` time if you actually want cross-host DDS.
ENV ROS_LOCALHOST_ONLY=1 \
    RCUTILS_COLORIZED_OUTPUT=1

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
