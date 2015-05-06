#Joystick Input For Control Of A System

## Overview

This ROS node feeds off of an inbound /joy topic containing joystick buttons and joysticks.  The node then can convert a joystick to the /cmd_vel or twist format outbound topic and can recognize hand controller buttons for other forms of system control

## Dependance on an inbound /joy topic

The system needs to feed this node with the /joy topic from a joystick of choice such as Xbox or Ps3 USB dongle and then the driver software.   Get the joystick drivers as below.

        sudo apt-get install ros-indigo-joystick-drivers

## Configuration of joystick in use

In this node's current configuration it is required to hand taylor the joystick button definitions into joy_input.cpp.

        Run the joystick driver using rosrun joy joy_node.
        Use rostopic echo /joy
        Inspect joy topic and find the buttons and joystick axis your controller generates.
        Modify the defines at the start of joy_input.cpp to match your controller

Yes, it would be nice to have this as some nice rosparam set to xbox or ps3 but that is not happening as of yet.

## Outputs of this node

This node will convert the joystick that has been configured to output to the /cmd_vel topic the classic 'twist' format topic which can then be picked up by other ROS nodes or the ROS Arduino Bridge depending on the system.

The node can be customized to output to other topics upon button activation.  
In this node's initial incantation 5 buttons have been chosen for STOP, FORWARD, RIGHT, LEFT, RESET and ABORT

## Direct serial output bypassing ROS output to /cmd_vel topic

This node in it's initial form will output the buttons directly on the serial port defined in the start of the joy_input.cpp file instead of to /cmd_vel topic but ONLY if the buttons are hit when a second BUTTON_HW_DIRECT is held.  This allows debug directly to PiBot motor drivers and bypassing ROS Arduino Bridge for debug.


