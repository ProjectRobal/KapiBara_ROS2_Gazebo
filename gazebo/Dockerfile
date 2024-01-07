FROM osrf/ros:iron-desktop

SHELL ["/bin/bash", "-c"]


ENV DEBIAN_FRONTEND noninteractive

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