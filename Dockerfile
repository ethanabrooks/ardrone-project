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

RUN pip --no-cache-dir install tensorflow gym[atari] opencv-python universe scipy

# catkin
COPY catkin/src/ArdroneRL catkin/src/ArdroneRL
RUN cp /bin/bash /bin/sh
RUN cd catkin/src && . /opt/ros/kinetic/setup.bash && catkin_init_workspace
RUN cd catkin && . /opt/ros/kinetic/setup.bash && catkin_make

COPY catkin/src/a3c catkin/src/a3c
RUN cd catkin && . /opt/ros/kinetic/setup.bash && catkin_make

# agent
#RUN pip install -e catkin/src/agent
RUN echo ". /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo ". /catkin/devel/setup.bash" >> /root/.bashrc
#RUN echo "f () { roslaunch ardrone_controller train.launch agent:=a3c \
      #gui:=false args:='--log-dir /tmp/ --env-id gazebo --num-workers $num_workers \
      #--job-name worker --task $i --remotes $remotes' }" >> /root/.bashrc


# TensorBoard
EXPOSE 6006

# xvfb
ENV DISPLAY :0

WORKDIR /root

# See http://answers.gazebosim.org/question/8065/unable-to-create-depthcamerasensor-when-launching-in-remote-computer/
RUN echo "Xvfb -shmem -screen 0 1280x1024x24 &" >> /root/.bashrc
#RUN echo 'function f { roslaunch a3c train.launch gui:=false \
     #num_workers:=$num_workers i:=$i remotes:=$remotes ; }' >> /root/.bashrc
RUN echo 'roslaunch a3c train.launch gui:=false \
     num_workers:=$num_workers i:=$i remotes:=$remotes' >> /root/.bashrc
CMD ["/bin/bash"]

