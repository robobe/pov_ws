FROM pov:dev

ARG VERSION=0.0.1


RUN echo "POV working dev: ${VERSION}">/etc/rome_working_dev



ENV DEBIAN_FRONTEND=noninteractive

# dev
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      python-is-python3 \
      ros-humble-rqt-robot-steering \
      rviz \
      ros-humble-interactive-markers \
      ros-humble-imu-tools \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# runtime
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      tmux \
      ros-humble-robot-localization \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*


ENV DEBIAN_FRONTEND=dialog

USER user


