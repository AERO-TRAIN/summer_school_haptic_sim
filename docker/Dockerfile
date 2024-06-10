FROM manfer/aerotrain_sim
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws
RUN mkdir -p ~/catkin_ws/src/summer_school_haptic_sim ; git clone --recursive https://github.com/AERO-TRAIN/summer_school_haptic_sim src/summer_school_haptic_sim ; catkin build

WORKDIR /root/
