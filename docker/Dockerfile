FROM ros:humble-ros-base-jammy

ARG PARENTDIR

WORKDIR /root/utfr_ws

USER root

RUN apt update && apt install -y ranger vim curl docker.io &&\
    # Install Required Libraries
    apt-get install -y git libboost-all-dev libserial-dev libusb-1.0-0.dev &&\
    apt-get install -y python3-pip &&\
    apt-get install -y ros-humble-rqt ros-humble-rqt-common-plugins 

# Add source to .bashrc:
RUN echo 'source /root/utfr_ws/scripts/utfr_source' >> /root/.bashrc

# Add build paths for ament and cmake, and sim:
RUN echo 'export EUFS_MASTER=/root/utfr_ws/src/utfr_dv_sim/' >> /root/.bashrc

# Downgrade setuptools to prevent build warnings:
RUN pip3 install setuptools==58.2.0

CMD ["bash"]
