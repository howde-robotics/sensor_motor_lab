#!/bin/sh
sudo chmod 666 /dev/ttyACM0
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
echo Start the GUI separately
