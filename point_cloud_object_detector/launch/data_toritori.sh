#!/bin/bash

for i in `seq 1 6`
do
    roslaunch point_cloud_object_detector point_cloud_object_detector.launch roomba:=$i
    sleep 2
done
