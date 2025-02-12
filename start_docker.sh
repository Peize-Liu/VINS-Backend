#!/bin/bash
docker run -it --runtime=nvidia --name vins_gtsam --gpus all --net=host \
  -v /home/dji/workspace/VINS-Backend:/root/catkin_ws/src/VINS-Fusion \
  -v /home/dji/ssd_1t_0/data_set/:/root/data_set \
  ros:vins-fusion /bin/bash