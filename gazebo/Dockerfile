FROM osrf/ros:iron-desktop

SHELL ["/bin/bash", "-c"]


ENV DEBIAN_FRONTEND noninteractive

COPY ./dependencies /app/dep

RUN --mount=type=cache,target=/var/cache/apt apt update -y

RUN --mount=type=cache,target=/var/cache/apt xargs apt -y install < /app/dep/packages.txt

RUN --mount=type=cache,target=/root/.cache/pip xargs pip3 install < /app/dep/python.txt

RUN mkdir -p /app/src

RUN mkdir -p /app/cmd

RUN mkdir -p /app/src/workspace

COPY --chmod=755 ./entrypoint.sh /entrypoint.sh

RUN chmod a+x ./entrypoint.sh
RUN chmod -R a+x /app/cmd

CMD [ "./entrypoint.sh" ]