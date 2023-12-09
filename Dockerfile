FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y
RUN apt install locales locales-all -y

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe

RUN apt install curl -y

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

RUN apt install openssh-server -y
RUN apt install x11vnc xvfb -y
RUN apt install ros-foxy-ros-base python3-argcomplete -y
RUN apt install ros-dev-tools -y
RUN apt install python3-colcon-common-extensions -y

RUN mkdir -p /app/src

RUN mkdir -p /app/src/workspace

COPY ./Docker/entrypoint.sh /entrypoint.sh
COPY ./Docker/create_workspace.sh /usr/local/bin/create_workspace.sh
COPY ./Docker/create_package.sh /usr/local/bin/create_package.sh
COPY ./Docker/build_packages.sh /usr/local/bin/build_packages.sh
COPY ./Docker/install_package.sh /usr/local/bin/install_package.sh


RUN chmod a+x ./entrypoint.sh
RUN chmod a+x /usr/local/bin/create_workspace.sh
RUN chmod a+x /usr/local/bin/create_package.sh
RUN chmod a+x /usr/local/bin/build_packages.sh
RUN chmod a+x /usr/local/bin/install_package.sh

CMD [ "./entrypoint.sh" ]