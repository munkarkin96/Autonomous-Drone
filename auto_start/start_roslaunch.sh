# !/bin/bash
LOG_FILE=/home/mun/autoStart/logs/log_start_roslaunch.txt
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running start_roslaunch.sh" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e

{
source /opt/ros/kinetic/setup.bash
source /home/mun/catkin_ws/devel/setup.bash

export ROS_WORKSPACE=/home/mun/catkin_ws

export ROS_MASTER_URI=http://mun-desktop:11311/
export ROS_IP=172.22.72.191

sleep 8

} &>> ${LOG_FILE}

set -v

{

roslaunch drone_development droneTest.launch

} &>> ${LOG_FILE}
