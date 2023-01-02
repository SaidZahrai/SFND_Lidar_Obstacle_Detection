FROM ubuntu:20.04

ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

COPY ./keyboard /etc/default/keyboard

ENV DEBIAN_FRONTEND noninteractive
# ENV LIBGL_ALWAYS_INDIRECT=1

RUN apt-get update
RUN apt-get install -y firefox build-essential cmake git libpcl-dev

RUN apt-get install xauth -y

RUN apt-get install xserver-xorg-video-intel
RUN apt-get install mesa-utils pcl-tools

# Replace 1000 with your user / group id
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    mkdir -p /etc/sudoers.d && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

USER developer
ENV HOME /home/developer
CMD /bin/bash
