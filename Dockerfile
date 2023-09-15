FROM ros:humble

RUN apt-get update
RUN apt-get upgrade -y
RUN apt -y install python3-pip

# Install python packages
RUN python3 -m pip install bosdyn-client==3.3.0 bosdyn-mission==3.3.0 bosdyn-choreography-client==3.3.0
RUN python3 -m pip install setuptools==58.2.0

# Install deps
RUN apt install -y ros-humble-velodyne
RUN apt install -y ros-humble-pcl-conversions
RUN apt install -y libopencv-dev
RUN apt install -y wget
 
# Install GTSAM library
WORKDIR /libraries
RUN wget https://github.com/borglab/gtsam/archive/refs/tags/4.2a8.tar.gz
RUN tar -xvzf 4.2a8.tar.gz
WORKDIR /libraries/gtsam-4.2a8/build
RUN cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
RUN make install
RUN ldconfig

# Install octomap & dynamicEDT3D
RUN apt -y install ros-humble-octomap-*
# RUN apt -y install ros-humble-dynamic-edt-3d

WORKDIR /libraries
RUN wget https://github.com/OctoMap/octomap/archive/refs/tags/v1.9.6.tar.gz
RUN tar -xvzf v1.9.6.tar.gz
WORKDIR /libraries/octomap-1.9.6/build
RUN cmake -DBUILD_OCTOVIS_SUBPROJECT=OFF ..
RUN make
RUN sudo make install

WORKDIR /ros2_ws/src
COPY aut_* .

RUN . /opt/ros/humble/setup.sh && \
	colcon build --packages-select aut_common aut_global_planner aut_launch aut_lidar_odometry aut_local_planner aut_localization aut_msgs aut_slam aut_spot aut_utils
RUN echo '. install/setup.bash' >> bootstrap.bash
RUN echo 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> bootstrap.bash
RUN chmod +x bootstrap.bash

CMD ["bash", "bootstrap.bash"]
