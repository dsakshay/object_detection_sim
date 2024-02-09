#!/bin/bash

docker run -dt --name ros1m_ouster --restart unless-stopped \
  --network host \
  --ipc=host\
  --privileged \
  --gpus all \
  -e ROS_IP=192.168.1.99 \
  -e ROS_MASTER_URI=http://192.168.1.99:11311 \
  -w /workspace \
  -v /home/companion/catkin_ws:/workspace \
  -v /etc/localtime:/etc/localtime:ro \
  -v /dev:/dev \
  -v /var/log:/core \
  -v /home/companion:/home/companion \
 nolan.azurecr.io/terramax-nav:ros1m_ouster /bin/bash

