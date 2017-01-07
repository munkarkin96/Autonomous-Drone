# !/bin/bash
LOG_FILE=/home/mun/autoStart/logs/logAutoStart.txt

echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running autostart_screens.sh" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e
set -v

{
sleep 2
screen -d -m bash /home/mun/autoStart/start_roscore.sh
sleep 4
screen -d -m bash /home/mun/autoStart/start_roslaunch.sh


} &>> ${LOG_FILE}
