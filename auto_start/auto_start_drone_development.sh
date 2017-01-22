#!/bin/bash
LOG_FILE=/home/mun/autoStart/logs/log_drone_development.txt
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running auto_start_drone_development.sh" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e

{
source /opt/ros/kinetic/setup.bash
source catkin_ws/devel/setup.bash


export ROS_WORKSPACE=/home/mun/catkin_ws

export ROS_MASTER_URI=http://mun-desktop:11311/
#export ROS_IP=172.22.54.192

sleep 5 
} &>> ${LOG_FILE}

set -v

{
roslaunch drone_development drone_start.launch
script output.txt


} &>> ${LOG_FILE}



