# syntax=docker/dockerfile:experimental

# Use experimental buildkit for faster builds
# https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/experimental.md
# Use `--progress=plain` to use plane stdout for docker build
#
# Example build command:
# export DOCKER_BUILDKIT=1
# export FROM_IMAGE="ros:foxy"
# export OVERLAY_MIXINS="release ccache"
# docker build -t lgsvl/ros2-lgsvl-bridge:foxy \
#   --build-arg FROM_IMAGE \
#   --build-arg OVERLAY_MIXINS \
#   -f Dockerfile ./

ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./lgsvl/ros2-lgsvl-bridge

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for unzipping
FROM $FROM_IMAGE AS unzipper
ARG DEBIAN_FRONTEND=noninteractive

# install helper dependencies
RUN apt-get update && apt-get install -q -y \
        wget \
        unzip

WORKDIR /tmp/lgsvl
ARG LGSVL_VERSION=2020.06
RUN wget -q -O lgsvlsimulator.zip \
        "https://github.com/lgsvl/simulator/releases/download/${LGSVL_VERSION}/lgsvlsimulator-linux64-${LGSVL_VERSION}.zip" && \
    unzip lgsvlsimulator.zip && \
    mv "lgsvlsimulator-linux64-${LGSVL_VERSION}" \
        simulator && \
    rm lgsvlsimulator.zip

# multi-stage for building
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

# edit apt for caching
RUN cp /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/docker-clean

# install lgsvl dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -q -y \
        ca-certificates \
        ccache \
        jq \
        lcov \
        libgl1 \
        libgtk2.0-0 \
        libvulkan1 \
        libx11-6 \
        libxau6 \
        libxcb1 \
        libxdmcp6 \
        libxext6 \
        unzip \
        wget \
    && rosdep update

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS \
      --symlink-install

# install lgsvl simulator
COPY --from=unzipper /tmp/lgsvl/simulator /opt/lgsvl/simulator
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,display
ADD "https://gitlab.com/nvidia/container-images/vulkan/raw/master/nvidia_icd.json" /etc/vulkan/icd.d/nvidia_icd.json

# restore apt for docker
RUN mv /etc/apt/docker-clean /etc/apt/apt.conf.d/ && \
    rm -rf /var/lib/apt/lists/

# source overlay from entrypoint
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=Ture
RUN if [ -n "$RUN_TESTS" ]; then \
        . $OVERLAY_WS/install/setup.sh && \
        colcon test \
        && colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
