#!/bin/bash

clear

# run terminals concurrently
#xterm -title "Roscore" -hold -e "echo \"Running Roscore\" && roscore"&
xterm -title "Turtlebot Minimal Bringup" -hold -e "sleep 1 && roslaunch turtlebot_bringup minimal.launch" &
xterm -title "Turtlebot Gmapping Launcher" -hold -e "sleep 10 && roslaunch turtlebot_navigation gmapping_demo.launch" &
xterm -title "Turtlebot RViz Launcher" -hold -e "sleep 15 && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
#xterm -title "Turtlebot Key Commands" -hold -e "sleep 12 && roslaunch turtlebot_teleop keyboard_teleop.launch"
xterm -title "Turtlebot Comp1 Launcher" -hold -e "sleep 20 && rosrun mie443_contest1 contest1" &
