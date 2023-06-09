#!/usr/bin/env bash

# xhost + 

docker rm container_roscore

docker run -it --net=host \
	--name=roscore \
	--device=/dev:/dev \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume=/dev:/dev \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--env="XAUTHORITY=$XAUTH" \
	--volume="$XAUTH:$XAUTH" \
	--privileged --name \
	container_roscore osrf/ros:noetic-desktop-full bash -c "roscore"