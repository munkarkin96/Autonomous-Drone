----- pre-flight checklist ---- 

1. rostopic echo latitude & longitude to make sure that the location of the teddy bear is received via telemetry 
2. rostopic obstacleStatus to check if the teraranger sensors are responding to obstacles in sequence L, F, R 
3. rostopic echo terangerone4 to check if it is working 
4. run simulation_arduino_listener.py to check if the quad can follow a teddy bear location 
5. run simulation_teraranger_listener.py to check if the main_code.py can run without a problem 
6. run main_code.py to check if it can fly over a detected obstacle 
7. test out precision_landing() 
8. fly towards a real GPS location that the telemetry is sending in the simulation 
9. after trying out in simulation, change the port to /dev/ttyACM1 
10. edit crontab 
11. restart and double check if everything can be started 

* might also want to comment out obstacle_avoidance() for further testing 
* make sure the LED is red in the teddy bear 
* check if the altitude for precision_landing() is appropriate 
* might want to stand further away to avoid detecting us as obstacles 


----- to launch the nodes ----- 

roslaunch drone_development drone_start.launch 

* this will launch all 11 nodes including main_code.py 
* make sure the respawn = 'False' for main_code.py as we do not want this code to run over and over 

------ simulation ------- 

simulation_arduino_listener.py 
simulation_teraranger_listener.py 

* are used to replicate the values that will be produced by the real item 
* for the ease of simulation in Gazebo SITL 


---- main_code.py ----- 

import all the my libraries 
check for FIXME 
check for TODO 
modify the QUICKFIX if time allows 

---- crontab ----- 

open up crontab.txt for the syntax 


---- autoStart ----

check the folder autoStart for the shell scripts to start roslaunch at boot up 
where log files is stored 


