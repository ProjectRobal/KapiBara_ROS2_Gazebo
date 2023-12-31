FROM ubuntu:22.04

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
RUN apt update -y

RUN apt install openssh-server -y
RUN apt install x11vnc xvfb -y
RUN apt install ros-iron-desktop python3-argcomplete -y
RUN apt install ros-dev-tools -y
RUN apt install python3-colcon-common-extensions -y

COPY ./dependencies /app/dep

RUN apt update -y

RUN xargs apt -y install < /app/dep/packages.txt

RUN mkdir -p /app/src

RUN mkdir -p /app/cmd

RUN mkdir -p /app/src/workspace

COPY --chmod=755 ./entrypoint.sh /entrypoint.sh

RUN chmod a+x ./entrypoint.sh
RUN chmod -R a+x /app/cmd

CMD [ "./entrypoint.sh" ]