#!/usr/bin/env bash

xhost + 

docker rm container_rviz

docker run -it --net=host \
	--name=rviz \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume=/dev:/dev \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--env="XAUTHORITY=$XAUTH" \
	--volume="$XAUTH:$XAUTH" \
	--privileged --name \
	container_rviz osrf/ros:noetic-desktop-full bash -c "export ROS_MASTER_URI=http://192.168.117.119:11311 && rosrun rviz rviz"
