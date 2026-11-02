# syntax=docker/dockerfile:1
# All images of the project. Build context is the repository root.
#
#   target   platform  image                         purpose
#   pc       amd64     ghcr.io/kpi-rover/pc          simulation, RViz, building the workspace on a PC
#   robot    arm64     ghcr.io/kpi-rover/robot       runs on the robot, starts on boot
#   robot-dev arm64    ghcr.io/kpi-rover/robot-dev   robot image + compilers, builds an overlay on the robot
#
# Sources are never kept in the pc image: the workspace is mounted from the host.

ARG ROS_DISTRO=jazzy
ARG ZELLIJ_VERSION=0.45.1
# Simulation-only packages (listed in kpi_rover_bringup), not installed on the robot
ARG SIM_ONLY_KEYS="ros_gz_sim ros_gz_bridge ros_gz_image gz_ros2_control"


# ---- deps-src: external packages from deps.repos ----
FROM ros:${ROS_DISTRO}-ros-base AS deps-src
RUN apt-get update && \
    apt-get install -y --no-install-recommends git python3-vcstool && \
    rm -rf /var/lib/apt/lists/*
COPY deps.repos /tmp/deps.repos
RUN mkdir /deps && \
    vcs import --input /tmp/deps.repos /deps && \
    find /deps -name .git -prune -exec rm -rf {} +


# ---- manifests: only package.xml and ignore markers, so dependency layers are not invalidated by code changes ----
FROM ros:${ROS_DISTRO}-ros-base AS manifests
COPY --from=deps-src /deps /src
COPY . /src
RUN find /src ! -type d ! -name package.xml ! -name COLCON_IGNORE ! -name AMENT_IGNORE ! -name CATKIN_IGNORE -delete


# ---- tools: zellij binary for the interactive images ----
FROM ros:${ROS_DISTRO}-ros-base AS tools
ARG ZELLIJ_VERSION
RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*
RUN ARCH=$(uname -m) && \
    curl -fsSL "https://github.com/zellij-org/zellij/releases/download/v${ZELLIJ_VERSION}/zellij-${ARCH}-unknown-linux-musl.tar.gz" \
      | tar xz -C /usr/local/bin zellij


# ---- pc: simulation and development on a PC (service `pc`) ----
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS pc
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-vcstool git nano vim less htop curl iputils-ping net-tools mesa-utils && \
    rm -rf /var/lib/apt/lists/*
COPY --from=manifests /src /tmp/src
RUN apt-get update && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths /tmp/src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/* /tmp/src
COPY --from=tools /usr/local/bin/zellij /usr/local/bin/zellij
COPY .zellij /etc/zellij
COPY docker/entrypoint.sh /usr/local/bin/kpi-entrypoint
COPY docker/bashrc /etc/kpi-rover.bashrc
RUN echo ". /etc/kpi-rover.bashrc" >> /etc/bash.bashrc
ENV KPI_WS=/workspace KPI_PROMPT=pc ZELLIJ_CONFIG_DIR=/etc/zellij \
    TERM=xterm-256color LANG=C.UTF-8 LC_ALL=C.UTF-8 SHELL=/bin/bash RCUTILS_COLORIZED_OUTPUT=1
WORKDIR /workspace
ENTRYPOINT ["kpi-entrypoint"]
CMD ["zellij"]


# ---- build-deps: build dependencies of the robot software ----
FROM ros:${ROS_DISTRO}-ros-base AS build-deps
ARG SIM_ONLY_KEYS
COPY --from=manifests /src /tmp/src
RUN apt-get update && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths /tmp/src --ignore-src -y --skip-keys "$SIM_ONLY_KEYS" && \
    rm -rf /var/lib/apt/lists/* /tmp/src


# ---- build: compiles the workspace into /ws/install ----
FROM build-deps AS build
WORKDIR /ws
COPY --from=deps-src /deps /ws/src
COPY . /ws/src
# build/ is a BuildKit cache mount kept between builds on the same machine, so only changed files are recompiled
RUN --mount=type=cache,target=/ws/build,id=kpi-rover-colcon-build \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF


# ---- robot-dev: robot image + compilers, builds an overlay on top of /ws/install (service `robot-dev`) ----
FROM build-deps AS robot-dev
RUN apt-get update && \
    apt-get install -y --no-install-recommends nano less htop iputils-ping net-tools && \
    rm -rf /var/lib/apt/lists/*
COPY --from=tools /usr/local/bin/zellij /usr/local/bin/zellij
COPY .zellij /etc/zellij
COPY docker/entrypoint.sh /usr/local/bin/kpi-entrypoint
COPY docker/bashrc /etc/kpi-rover.bashrc
RUN echo ". /etc/kpi-rover.bashrc" >> /etc/bash.bashrc
# Same path as in the build stage: colcon setup scripts contain absolute prefixes
COPY --from=build /ws/install /ws/install
ENV KPI_WS=/overlay KPI_PROMPT=robot-dev ZELLIJ_CONFIG_DIR=/etc/zellij \
    TERM=xterm-256color LANG=C.UTF-8 LC_ALL=C.UTF-8 SHELL=/bin/bash RCUTILS_COLORIZED_OUTPUT=1
WORKDIR /overlay
ENTRYPOINT ["kpi-entrypoint"]
CMD ["zellij"]


# ---- robot: runtime dependencies and the built workspace only (service `robot`) ----
FROM ros:${ROS_DISTRO}-ros-base AS robot
ARG SIM_ONLY_KEYS
COPY --from=manifests /src /tmp/src
RUN apt-get update && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths /tmp/src --ignore-src -y --dependency-types=exec --skip-keys "$SIM_ONLY_KEYS" && \
    rm -rf /var/lib/apt/lists/* /tmp/src
COPY docker/entrypoint.sh /usr/local/bin/kpi-entrypoint
COPY --from=build /ws/install /ws/install
ENV RCUTILS_COLORIZED_OUTPUT=1
ENTRYPOINT ["kpi-entrypoint"]
CMD ["ros2", "launch", "kpi_rover_bringup", "robot.launch.py"]
