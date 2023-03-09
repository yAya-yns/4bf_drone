FROM melodic-ros-base-bionic

ARG PARENTDIR

WORKDIR /root/catkin_ws

USER root

RUN apt update && apt install -y ranger git vim curl pip &&\
    #sh -c 'echo "deb http://packages.ros.org/ros/ubuntu \
    #$(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    #curl -s \
    #https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    #sudo apt-key add - \ &&
    #apt update &&\
    DEBIAN_FRONTEND=noninteractive apt install -y ros-melodic-joy ros-melodic-teleop-twist-joy \
    ros-melodic-desktop-full \ 
    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
    ros-melodic-rgbd-launch ros-melodic-rosserial-arduino \
    ros-melodic-rosserial-python ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
    ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-compressed-image-transport ros-melodic-rqt* \
    ros-melodic-rviz ros-melodic-gmapping \
    ros-melodic-gazebo-ros \
    ros-melodic-tf-conversions \
    ros-melodic-navigation ros-melodic-interactive-markers \
    ros-kinetic-mavros ros-kinetic-mavros-extras \
    && 
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh &&\
    sudo bash ./install_geographiclib_datasets.sh   


# Add source to .bashrc:
RUN echo 'source /root/catkin_ws/scripts/source' >> /root/.bashrc

CMD ["bash"]
