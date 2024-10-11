FROM althack/ros2:iron-cuda-gazebo
# FROM althack/ros2:iron-gazebo


SHELL ["/bin/bash", "-c"]


ENV DEBIAN_FRONTEND noninteractive

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY ./dependencies /app/dep

RUN --mount=type=cache,target=/var/cache/apt apt update -y

RUN --mount=type=cache,target=/var/cache/apt xargs apt -y install < /app/dep/packages.txt

#RUN --mount=type=cache,target=/root/.cache/pip xargs pip3 install < /app/dep/python.txt

RUN mkdir -p /app/src

RUN mkdir -p /app/cmd

RUN mkdir -p /app/src/workspace

#COPY --chmod=755 ./entrypoint.sh /entrypoint.sh


#RUN chmod a+x ./entrypoint.sh
RUN chmod -R a+x /app/cmd

CMD [ "./entrypoint.sh" ]