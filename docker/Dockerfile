FROM ros:lyrical-ros-base

# Make sure we are using bash and catching errors
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
# ------------------------------------------------------------------------------
# Build configuration
#
# EXACT_FP=ON          builds with -O2 -fno-fast-math -ffp-contract=off instead
#                      of -Ofast, so that results are reproducible across
#                      compilers at the cost of some speed.
# DETERMINISTIC_FFT=ON makes FFTW select its plan by heuristic instead of by
#                      timing itself, so that results are reproducible across
#                      runs.
#
# Both default to OFF, which is the configuration this package ships with.
# ------------------------------------------------------------------------------
ARG EXACT_FP=OFF
ARG DETERMINISTIC_FFT=OFF
# ------------------------------------------------------------------------------
# Migrate to new ROS sign key
# https://github.com/osrf/docker_images/issues/807#issuecomment-2927469824
# ------------------------------------------------------------------------------
RUN apt-get update \
  && apt-get install -y ca-certificates curl
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
# python3-argcomplete and python3-colcon-cd are named explicitly because the
# bashrc files copied in below expect the register-python-argcomplete command
# and colcon_cd.sh, and neither ships as part of the base image's own colcon
# installation.
RUN apt-get -y update                                                       && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo apt-utils build-essential g++ git curl lsb-release pkg-config \
    python3-rosdep python3-argcomplete python3-colcon-cd \
    ros-lyrical-rmw-cyclonedds-cpp gosu clang \
    bash-completion vim nano bc                                             && \
    rm -rf /var/lib/apt/lists/*
# rosdep init is deliberately not run. The base image already ships
# /etc/ros/rosdep/sources.list.d/20-default.list, and its content is byte for
# byte what rosdep init writes, so deleting it and fetching an identical copy
# bought nothing and cost a network round trip that has no retry behind it. One
# rate-limited answer from the host it reads failed the whole build, with a
# Python traceback rather than a sentence. rosdep update below still needs the
# network, and that one is unavoidable.
################################################################################
# User configuration
################################################################################
# Use bash
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# ------------------------------------------------------------------------------
# Create new user and home directory
# https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
ARG USER=fsm_lidar_odometry
RUN useradd --create-home ${USER}                                           && \
    echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER}         && \
    chmod 0440 /etc/sudoers.d/${USER}                                       && \
    mkdir -p /home/${USER}
# ------------------------------------------------------------------------------
# copy .bashrc
COPY --chown=${USER}:${USER} docker/bashrc      /home/${USER}/.bashrc
COPY --chown=${USER}:${USER} docker/bashrc_base /home/${USER}/.bashrc_base
COPY --chown=${USER}:${USER} docker/bashrc_ros2 /home/${USER}/.bashrc_ros2
# ------------------------------------------------------------------------------
# Bind mount/shared directory between container and host
# https://docs.docker.com/storage/bind-mounts/
# ------------------------------------------------------------------------------
USER ${USER}
RUN mkdir /home/${USER}/shared-input
RUN mkdir /home/${USER}/shared-output
USER root
################################################################################
# ROS 2 lyrical configuration
################################################################################
RUN rosdep update

COPY --chown=${USER}:${USER} fsm_lidar_odometry/ /home/${USER}/ros2_ws/src/fsm_lidar_odometry/

# The manifest is the one list of what this package needs. Resolve it rather
# than repeating it here, so that a dependency added to the manifest cannot be
# forgotten in this file.
#
# The libfftw3 key is the exception. Its rule in ros/rosdistro resolves to
# libfftw3-3, which Ubuntu dropped in favour of per-precision packages and
# which therefore does not exist on this release. Until that rule carries an
# override, the key is skipped and the development package, which pulls the
# runtime it needs, is named directly.
RUN apt-get -y update                                                       && \
    apt-get install -y --no-install-recommends libfftw3-dev                 && \
    rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -r -y   \
      --skip-keys libfftw3                                                  && \
    rm -rf /var/lib/apt/lists/*

# Drop to ${USER} so that `ros2_ws` and everything under it are owned by ${USER}
USER ${USER}

RUN source /opt/ros/lyrical/setup.bash                                      && \
    cd /home/${USER}/ros2_ws                                                && \
    colcon build                                                               \
      --cmake-args                                                             \
        -DFSM_LIDAR_ODOMETRY_EXACT_FP=${EXACT_FP}                                          \
        -DFSM_LIDAR_ODOMETRY_DETERMINISTIC_FFT=${DETERMINISTIC_FFT}

# Restore default user
USER root
# ------------------------------------------------------------------------------
# Entrypoint stuff
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} docker/entrypointd.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ------------------------------------------------------------------------------
WORKDIR /home/${USER}
CMD ["/bin/bash"]
