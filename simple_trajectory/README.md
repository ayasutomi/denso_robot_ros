# HOW TO RUN

Run simple_trajectory after bringing up the vs060 simulation in gazebo using the two commands below:
```
roslaunch denso_robot_bringup vs060_bringup.launch sim:=true
rosrun simple_trajectory simple_trajectory
```
It is possible to use the simple_trajectory with the real robot too with the command below, BUT DOUBLE CHECK FIRST THE ROBOT MOVEMENT IN GAZEBO BEFORE MOVING THE REAL ROBOT.
```
roslaunch denso_robot_bringup vs060_bringup.launch sim:=false ip_address:=192.168.0.1
```