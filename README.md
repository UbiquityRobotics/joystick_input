#Joystick Input For Control Of A System

## Overview

This ROS node feeds off of an inbound /joy topic containing joystick buttons and joysticks.  The node then can convert a joystick to the /cmd_vel or twist format outbound topic and can recognize hand controller buttons for other forms of system control

## Dependance on an inbound /joy topic

The system needs to feed this node with the /joy topic from a joystick of choice such as Xbox or Ps3 USB dongle and then the driver software.   Get the joystick drivers as below.

        sudo apt-get install ros-indigo-joystick-drivers

## Dependance on navigation stack

The node now has ability to use  movebase specification to nav targets so we need that code

        sudo apt-get install  ros-indigo-navigation

## Configuration of joystick in use

The node was written to support XBox 360 controller and PS3 bluetooth controller but the PS3 dongle needs to be able to be set to emulate XBox and only then will you have luck.   We have found that the  'Wireless PS3 Controller To PC USB Adapter [by Mayflash] works if set to Xinput when using PS3 controller.  The XBox dongle we have found to work is the 'PC Wireless Gaming Receiver' which says model 1086 on the tag near the usb plug.

        Run the joystick driver using:   roslaunch joy_input joystick.launch
        To see the drivers /joy topic:   rostopic echo /joy
        Inspect joy topic and find the buttons and joystick axis your controller generates.
        Modify the defines at the start of joy_input.cpp to match your controller if required.

## Manual operation without a joystick

You can issue messages manually to the /joy topic where first button is 0 (below is a single line!)

rostopic pub /joy sensor_msgs/Joy '{ header: {seq: 10, stamp: {secs: 1431222430, nsecs: 345678}, frame_id: "3"},
   axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'

## Definition of button mapping to functionality

Here we explain what the buttons do.  Both the XBOX and PS3 have same physical layout for the keys we use but may look a little different so I'll try to explain the buttons for both units.   There are 3 special buttons that either act as enable or function 'shift' buttons that are held when the for colorful keys to the right hand are pushed.

Left Joystick (only when left front top button is held will the joystick operate)
  This joystick controls motion for forward, back and right to left.  Output is only to /cmd_vel topic or the 'twist' messages for classic bot control.  You can move joystick and just click the left top front button or hold button and move stick.

## The 4 multi-function colored keys

These 4 keys each have 3 functions depending on if any 'mode shift keys' are being held.

With no 'mode shift keys held' the 4 buttons will issue a fixed simple set of velocities to the /cmd_vel topic.   Default speeds will be used for these simple move functions.  A couple ros params will set these simple speeds.

Forward:  The XBox yellow 'Y' (PS3 triangle)
Stop:     The XBox green  'A' (PS3 X)
Right:    The XBox red    'B' (PS3 circle)
Left:     The XBox blue   'X' (PS3 square)


## Direct Hardware Drive   
The XBox 'Start' (PS3 Start) is the Direct Hardware Drive 'mode shift key' held down the above 4 movement commands are issued as if a ROS Arduino bridge is hooked up to the serial port.  The directions will be as above. This was included so you can run the bot without ROS Arduino Bridge.  Some of our robots will not do anything in this mode as without ROS Arduino Bridge the serial goes nowhere. 


## Navigation Target Specification   
Uses XBox 'Back' button (PS3 'Select' button) as a Nav target specification 'mode shift key'.

When held down this causes the 4 colorful keys to issue movebase commands to the 4 configured targets.  Target definitions are given as x,y,w as ROS parameters in the launch file but will be re-read every 5 seconds or so to allow dynamic changing of target nav points as the bot is running.  You must use  rosparam set to change those if already running.

Target1:  The XBox yellow 'Y' (PS3 triangle)
Target2:  The XBox red    'B' (PS3 circle)
Target3:  The XBox blue   'X' (PS3 square)
Target4:  The XBox green  'A' (PS3 X)

        
