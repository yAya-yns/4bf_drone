FROM ros:melodic-ros-core-bionic

ARG PARENTDIR

WORKDIR /root/catkin_ws

USER root

RUN apt update && apt install -y ranger git vim curl python3-pip \
    #sh -c 'echo "deb http://packages.ros.org/ros/ubuntu \
    #$(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    #curl -s \
    #https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    #sudo apt-key add - \ &&
    #apt update &&\
    ros-melodic-desktop-full \ 
    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
    ros-melodic-rgbd-launch \
    ros-melodic-rosserial-python ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
    ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-compressed-image-transport ros-melodic-rqt* \
    ros-melodic-rviz ros-melodic-gmapping \
    ros-melodic-gazebo-ros \
    ros-melodic-tf-conversions \
    ros-melodic-navigation ros-melodic-interactive-markers \
    ros-melodic-mavros ros-melodic-mavros-extras 


# Add source to .bashrc:
RUN echo 'source /root/catkin_ws/scripts/source' >> /root/.bashrc

CMD ["bash"]
