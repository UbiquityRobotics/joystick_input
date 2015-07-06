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

The parameters in joystick.launch are briefly discussed below but are commented in the launch file as well

        enable_joystick           Set to 0 to disable joystick completely (not likely to be needed)
        joystick_deadzone         Set this to change the deadzone where joystick returns 0 if lower 
        cmd_vel_msg_per_sec       Max messages issued per second to throttle too many messages
                                  This defaults to about 4 per second but you can set up to about 10.
        cmd_vel_speed             The speed used for the forward button (not the joystick)
        cmd_vel_angle             The angular speed used for the right or left button (not the joystick)
        cmd_vel_joy_max           The joystick fwd/back max value for joystick pressed all the way
        cmd_vel_joy_turn_max      The joystick right/left max value for joystick pressed all the way

        disable_nav_stack         Set to 0 to disable move_base hotkeys programmed with target_N_x, y, z
        target_N_x                Pre-programed move_base x,y,z values for wireless 'goto nav point' use

The joy_input node consumes topic /joy and you may find it of value see the output of the required ros-indigo-joystick-drivers node using this command below.  If you don't see anything on /joy then the joy_input node will have no input.

        rostopic echo /joy

It is not likely that you will have issues but if you have some custom joystick that is recognized and output does come out on /joy but is different than expected then inspect joy topic and find the buttons and joystick axis your controller generates.  If then required, modify the defines at the start of joy_input.cpp to match your controller as required.

## Running the Joystick Input Node

The node is started using roslaunch and you may modify the parameters discussed in the configuration section by use of rosparam list then rosparam set <paramter_name> <value>  at runtime because the parameters are refreshed in the node.

        roslaunch joy_input joystick.launch

The window that starts the node will show the published /cmd_vel values of linear.x and angular.z.
You may also find it of use to look at the /cmd_vel topic to see those values as follows

        rostopic echo /cmd_vel

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

        
