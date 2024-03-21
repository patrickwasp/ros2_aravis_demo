FROM ros:humble

# use bash instead of sh
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Set the entrypoint to launch foxglove bridge
CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]