#!/bin/bash
docker run -it --runtime=nvidia --name vins_backend --gpus all --net=host  -v /home/dji/workspace/VINS-Backend:/root/catkin_ws/src/VINS-Fusion   ros:vins-fusion    /bin/bash