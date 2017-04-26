FROM ros:kinetic

ENV DEBIAN_FRONTEND noninteractive

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

RUN apt-get update && apt-get install -y \
        build-essential \
        curl \
        wget \
        git \
        libfreetype6-dev \
        libpng12-dev \
        libzmq3-dev \
        pkg-config \
        python-dev \
        python-numpy \
        python-pip \
        software-properties-common \
        swig \
        zip \
        zlib1g-dev \
        rsync \
        vim \
        xvfb

RUN apt-get remove ros-kinetic-gazebo-ros-pkgs
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y gazebo8
RUN apt-get install -y \
      ros-kinetic-gazebo-ros-pkgs \
      ros-kinetic-ardrone-autonomy \
      ros-kinetic-robot-state-publisher \
      ros-kinetic-xacro \
      python-dev \
      make \
      golang \
      libjpeg-turbo8-dev \
      && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN curl -fSsL -O https://bootstrap.pypa.io/get-pip.py && \
    python get-pip.py && \
    rm get-pip.py

# go-vncdriver
COPY go-vncdriver go-vncdriver
RUN apt-get install -y python-dev make golang libjpeg-turbo8-dev
RUN pip install go-vncdriver

RUN pip --no-cache-dir install tensorflow 'gym[atari]==0.7.4' opencv-python universe scipy

# Install SSH server
RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd
RUN echo 'root:nothing' | chpasswd
RUN sed -i 's/PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config
RUN sed -i 's/Port 22/Port 22123/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

# TensorBoard
EXPOSE 6006

# xvfb
ENV DISPLAY :1.0

EXPOSE 22

RUN echo 'f () { roslaunch a3c train.launch gui:=false \
     num_workers:=$num_workers i:=$i remotes:=$remotes ; }' >> /root/.bashrc

RUN echo ". /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo ". /catkin/devel/setup.bash" >> /root/.bashrc

RUN cp /bin/bash /bin/sh 
RUN mkdir -p catkin/src
RUN cd catkin/src && . /opt/ros/kinetic/setup.bash && catkin_init_workspace

# tum_simulator
COPY catkin/src/tum_simulator/ catkin/src/tum_simulator
RUN . /opt/ros/kinetic/setup.bash && cd catkin && catkin_make

RUN apt-get install -y \
      telnet \
      nmap \
      iputils-ping \
      net-tools

# A3C
COPY catkin/src/a3c catkin/src/a3c
RUN cd catkin && . /opt/ros/kinetic/setup.bash && catkin_make
COPY start.sh /
