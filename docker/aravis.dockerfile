FROM ros:humble

# use bash instead of sh
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# install drivers and sdks
RUN apt-get update && apt-get install -y \
    aravis-tools \
    libaravis-0.8-0 \
    libaravis-dev \
    libaravis-doc \
    libgtk-3-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libnotify-dev \
    libglib2.0-dev \
    libxml2-dev \
    zlib1g-dev \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# copy ROS packages into container
COPY aravis_camera /ros2_ws/src/aravis_camera

# build ROS packages and allow non-compiled
# sources to be edited without rebuild
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

# add sourcing ros2_ws to entrypoint
RUN sed --in-place --expression \
    '$isource "/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

# add packages to path
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# copy udev rules
COPY drivers/udev/90-aravis.rules /etc/udev/rules.d/90-aravis.rules
# start udev to trigger rules to add devices
RUN sed --in-place \
    '/^exec "\$@"/i /lib/systemd/systemd-udevd --daemon && udevadm trigger' \
    /ros_entrypoint.sh


