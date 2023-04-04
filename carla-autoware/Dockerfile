ARG AUTOWARE_VERSION=1.14.0-melodic-cuda

FROM autoware/autoware:$AUTOWARE_VERSION

WORKDIR /home/autoware

# Update simulation repo to latest master.
COPY --chown=autoware update_sim.patch ./Autoware
ADD --chown=autoware patch/ ./Autoware/patch/
RUN patch ./Autoware/autoware.ai.repos ./Autoware/update_sim.patch
# CMD rm -rf ./Autoware/src
# ADD --chown=autoware Autoware/src ./
RUN cd ./Autoware \
    && vcs import src < autoware.ai.repos \
    && patch src/autoware/core_perception/roi_object_filter/src/roi_object_filter.cpp patch/roi_object_filter.cpp.patch \
    && patch src/autoware/core_perception/range_vision_fusion/src/range_vision_fusion.cpp patch/range_vision_fusion.cpp.patch \
    && patch src/autoware/core_planning/pure_pursuit/src/pure_pursuit_core.cpp patch/pure_pursuit_core.cpp.patch \
    && patch src/autoware/core_planning/lane_planner/nodes/lane_select/lane_select_core.cpp patch/lane_select_core.cpp.patch \
    && git --git-dir=./src/autoware/simulation/.git --work-tree=./src/autoware/simulation pull \
    && source /opt/ros/melodic/setup.bash \
    && AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# RUN echo "roslaunch carla_autoware_agent carla_autoware_agent.launch town:=Town03" > ./Autoware/test.sh

# CARLA PythonAPI
RUN mkdir ./PythonAPI
# ADD --chown=autoware https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.10-py2.7-linux-x86_64.egg ./PythonAPI
ADD --chown=autoware carla-api/carla-0.9.10-py2.7-linux-x86_64.egg ./PythonAPI
RUN echo "export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)" >> .bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:~/PythonAPI/\$PYTHON2_EGG" >> .bashrc

# COPY ./cuda-keyring_1.0-1_all.deb /tmp/
RUN sudo sed -i '/developer\.download\.nvidia\.com\/compute\/cuda\/repos/d' /etc/apt/sources.list
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb && sudo dpkg -i cuda-keyring_1.0-1_all.deb
RUN sudo apt-key del 7fa2af80 && sudo rm /etc/apt/sources.list.d/cuda.list && sudo rm /etc/apt/sources.list.d/nvidia-ml.list


# CARLA ROS Bridge
# There is some kind of mismatch between the ROS debian packages installed in the Autoware image and
# the latest ros-melodic-ackermann-msgs and ros-melodic-derived-objects-msgs packages. As a
# workaround we use a snapshot of the ROS apt repository to install an older version of the required
# packages.
RUN sudo rm -f /etc/apt/sources.list.d/ros1-latest.list
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN sudo sh -c 'echo "deb http://snapshots.ros.org/melodic/2020-08-07/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-snapshots.list'
RUN sudo apt-get update
RUN sudo apt-get install -y --no-install-recommends \
        python-pip \
        python-wheel \
        ros-melodic-ackermann-msgs \
        ros-melodic-derived-object-msgs
RUN sudo rm -rf /var/lib/apt/lists/*
RUN pip install simple-pid pygame networkx==2.2

RUN git clone -b '0.9.10.1' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# CARLA Autoware agent
COPY --chown=autoware . ./carla-autoware

RUN mkdir -p carla_ws/src
RUN cd carla_ws/src \
    && ln -s ../../ros-bridge \
    && ln -s ../../carla-autoware/carla-autoware-agent \
    && cd .. \
    && source /opt/ros/melodic/setup.bash \
    && catkin_make

RUN echo "export CARLA_AUTOWARE_CONTENTS=~/autoware-contents" >> .bashrc \
    && echo "source ~/carla_ws/devel/setup.bash" >> .bashrc \
    && echo "source ~/Autoware/install/setup.bash" >> .bashrc

CMD ["/bin/bash"]

COPY --chown=autoware ./entrypoint.sh /tmp
USER autoware
ENTRYPOINT ["/tmp/entrypoint.sh"]
