FROM ubuntu:14.04

RUN apt-get update
RUN apt install -y sudo python-pip curl git tmux screen wget software-properties-common unzip

ADD install_dependencies.sh /tmp/install_dependencies.sh
RUN /bin/bash /tmp/install_dependencies.sh

ENTRYPOINT bash