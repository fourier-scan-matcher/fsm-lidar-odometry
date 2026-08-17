FROM ros:lyrical-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ARG EXACT_FP=OFF
ARG DETERMINISTIC_FFT=OFF
RUN apt-get update \
  && apt-get install -y ca-certificates curl
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
RUN apt-get -y update                                                       && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo apt-utils build-essential g++ git curl lsb-release pkg-config \
    python3-rosdep python3-argcomplete python3-colcon-cd \
    ros-lyrical-rmw-cyclonedds-cpp gosu clang \
    bash-completion vim nano bc                                             && \
    rm -rf /var/lib/apt/lists/*
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
ARG USER=fsm_lidar_odometry
RUN useradd --create-home ${USER}                                           && \
    echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER}         && \
    chmod 0440 /etc/sudoers.d/${USER}                                       && \
    mkdir -p /home/${USER}
COPY --chown=${USER}:${USER} docker/bashrc      /home/${USER}/.bashrc
COPY --chown=${USER}:${USER} docker/bashrc_base /home/${USER}/.bashrc_base
COPY --chown=${USER}:${USER} docker/bashrc_ros2 /home/${USER}/.bashrc_ros2
USER ${USER}
RUN mkdir /home/${USER}/shared-input
RUN mkdir /home/${USER}/shared-output
USER root
RUN rosdep update

COPY --chown=${USER}:${USER} fsm_lidar_odometry/ /home/${USER}/ros2_ws/src/fsm_lidar_odometry/

RUN apt-get -y update                                                       && \
    apt-get install -y --no-install-recommends libfftw3-dev                 && \
    rosdep install --from-paths /home/${USER}/ros2_ws/src --ignore-src -r -y   \
      --skip-keys libfftw3                                                  && \
    rm -rf /var/lib/apt/lists/*

USER ${USER}

RUN source /opt/ros/lyrical/setup.bash                                      && \
    cd /home/${USER}/ros2_ws                                                && \
    colcon build                                                               \
      --cmake-args                                                             \
        -DFSM_LIDAR_ODOMETRY_EXACT_FP=${EXACT_FP}                                          \
        -DFSM_LIDAR_ODOMETRY_DETERMINISTIC_FFT=${DETERMINISTIC_FFT}

USER root
COPY --chown=${USER}:${USER} docker/entrypointd.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /home/${USER}
CMD ["/bin/bash"]
