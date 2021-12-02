#!/bin/bash

export HOME=/home/ubuntu
mkdir -p /ex31

echo "**** start logging ****" &> /ex31/build.log

apt -y update &>> /ex31/build.log
apt -y install docker.io &>> /ex31/build.log
usermod -aG docker root &>> /ex31/build.log

docker pull ${docker_image} &>> /ex31/build.log

# where the core dump goes is configured in the host (the EC2 instance) not the docker container
# this is saying it should go the to /cores folder
systemctl disable apport.service
mkdir -p /cores
echo '/cores/core.%e.%p' | tee /proc/sys/kernel/core_pattern

docker run --init --ulimit core=-1 --mount type=bind,source=/cores/,target=/cores/ -v /ex31:/ginan/examples/ex31 -e GINAN=${ginan_root} ${docker_image} /ginan/docker/run-tests-long.sh &>> /ex31/build.log

echo "**** end logging ****" &>> /ex31/build.log
