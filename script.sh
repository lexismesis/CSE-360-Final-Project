#!/bin/bash

# loads green stoplight at first "intersection"
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/colordetection/src/greenbox.urdf -urdf -x 9 -y 3.2 -z 10 -model traffic_light_green

# loads yellow stoplight at second "intersection"
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/colordetection/src/yellowbox.urdf -urdf -x -5.8 -y 9.2 -z 10 -model traffic_light_yellow

# loads green stoplight at third "intersection"
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/colordetection/src/redbox.urdf -urdf -x -18 -y 15 -z 10 -model traffic_light_red

# loads red stoplight at the fourth "intersection"
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/colordetection/src/yellowbox.urdf -urdf -x -16 -y 1 -z 10 -model traffic_light_yellow2
