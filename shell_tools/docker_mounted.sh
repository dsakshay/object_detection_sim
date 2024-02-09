#!/bin/bash
CONTAINER_NAME=$1

sudo docker run -it --rm \
	--privileged \
	--network host \
	--name $CONTAINER_NAME \
	--workdir /work \
	-v /home/akshay/es_ws:/work \
	nolan.azurecr.io/terramax-nav-ros2h_amd64:profiling_code /bin/bash

