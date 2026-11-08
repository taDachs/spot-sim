# Spot simulator: ROS 2 Jazzy + Gazebo Harmonic (vendor packages) + the Spot API plugins.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      pkg-config \
      libgrpc++-dev \
      libprotobuf-dev \
      protobuf-compiler \
      protobuf-compiler-grpc \
      ros-jazzy-gz-sim-vendor \
      ros-jazzy-gz-plugin-vendor \
      ros-jazzy-gz-tools-vendor \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Spot API definitions, pinned to the SDK version the driver targets.
ARG SPOT_SDK_VERSION=v5.0.1
RUN git clone --quiet --depth 1 --branch "${SPOT_SDK_VERSION}" --filter=blob:none --sparse \
      https://github.com/boston-dynamics/spot-sdk.git /opt/spot-sdk \
    && git -C /opt/spot-sdk sparse-checkout set protos \
    && rm -rf /opt/spot-sdk/.git
ENV SPOT_SDK_PROTO_DIR=/opt/spot-sdk/protos

WORKDIR /ws
ARG COLCON_ARGS="--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

# Generated Spot API code is the slow part; build it in its own layer.
COPY src/spot_sim_api_protos src/spot_sim_api_protos
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select spot_sim_api_protos ${COLCON_ARGS}

# Robot model (meshes, xacro) from the driver repository.
COPY spot_ros2/spot_description src/spot_description

COPY src/ src/
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-skip spot_sim_api_protos ${COLCON_ARGS}

# GUI support (gz sim -g via the host's X server).
RUN apt-get update && apt-get install -y --no-install-recommends xauth \
    && rm -rf /var/lib/apt/lists/*
COPY docker/with_x11.sh /usr/local/bin/with-x11

COPY docker/sim_entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "spot_sim_bringup", "sim.launch.py"]
