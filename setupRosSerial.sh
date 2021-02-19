#!/bin/bash
#Takes one optional agument for port to connect to arduino
#Defaults to /dev/ttyACM0
#may ask for permission to access port, require password for sudo
PORT=/dev/ttyACM0
echo Trying to connect through ${1:-$PORT}
sudo chmod 666 /dev/ttyACM0
gnome-terminal -- /bin/sh -c "roscore"
sleep 1
gnome-terminal -- /bin/sh -c "rosrun rosserial_python serial_node.py _port:=${1:-$PORT}"
#gnome-terminal -- 
./sensor_motor_lab_gui
