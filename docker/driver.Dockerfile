# spot_ros2 driver (unmodified, from the local submodule) on ROS 2 Humble.
# Mirrors spot_ros2/install_spot_ros2.sh, minus sudo.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-distutils \
      python3-apt \
      qttools5-dev \
      wget \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies of spot_wrapper (Spot Python SDK 5.0.1 etc.).
COPY spot_ros2/spot_wrapper/requirements.txt /tmp/spot_wrapper_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/spot_wrapper_requirements.txt \
    && pip3 install --no-cache-dir --force-reinstall "setuptools==59.6.0"

# Prebuilt bosdyn_msgs (ROS messages for the Spot API) and the Spot C++ SDK.
ARG SDK_VERSION=5.0.1
RUN ARCH="$(dpkg --print-architecture)" \
    && wget -q -O /tmp/bosdyn_msgs.run \
       "https://github.com/rai-opensource/bosdyn_msgs/releases/download/${SDK_VERSION}/ros-humble-bosdyn_msgs_${SDK_VERSION}-jammy_${ARCH}.run" \
    && chmod +x /tmp/bosdyn_msgs.run \
    && (set +o pipefail; yes | /tmp/bosdyn_msgs.run --nox11) \
    && wget -q -O /tmp/spot-cpp-sdk.deb \
       "https://github.com/rai-opensource/spot-cpp-sdk/releases/download/v${SDK_VERSION}/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb" \
    && dpkg -i /tmp/spot-cpp-sdk.deb \
    && rm -f /tmp/bosdyn_msgs.run /tmp/spot-cpp-sdk.deb

WORKDIR /ws
COPY spot_ros2/ src/spot_ros2/
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y -r --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-up-to spot_driver spot_rviz_plugins --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# GUI support (RViz via the host's X server).
RUN apt-get update && apt-get install -y --no-install-recommends xauth \
    && rm -rf /var/lib/apt/lists/*
COPY docker/with_x11.sh /usr/local/bin/with-x11

COPY docker/driver_entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
