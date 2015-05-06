/*
 * This is a node to receive joystick topic of /joy and act accordingly
 *
 * This node was formed by shameless copy of the wiki.ros.org/joy/Tutorials/WritingTeleopNode code
 * The node listens to topic /joy which should be supplied perhaps by wireless joystick node.
 *
 * joy buttons and joysticks then cause this node to put out control to the system topics
 *
 * Output to /cmd_vel classic twist commands for joystick axis defined below if BUTTON_CMD_VEL
 * is being held down.  This then allows release of all buttons to set speed to 0
 *
 * Output of other buttons can be mapped to bot specific needs such as 'abort' or 
 * other special functions.  Modify code looking for buttons for suitable changes
 *
 * Initial Code by Mark Johnston 
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>           // joy topic message format
#include <geometry_msgs/Twist.h>       // classic 'twist' messages for bot motion commands

#define THIS_NODE_NAME	"joy_input"    // Node name hard coded

// Locate these in a system-wide include which I am not aware of at this time
#define  SERIAL_DEV_TO_MOTOR_DRIVER   "/dev/ttyAMA0"    // Raspberry Pi main serial port
#define  WHEEL_SPEED_MAX              20                // max wheel speed

// Button definitions for the array place used for a given controller
// use rostopic echo /joy for your controller and define buttons below
#define BUTTON_STOP       0
#define BUTTON_RESET      5
#define BUTTON_FORWARD    3
#define BUTTON_RIGHT      1
#define BUTTON_LEFT       2
#define BUTTON_CMD_VEL    4     // When held we will publish twist messages to /cmd_vel on joystick
#define BUTTON_HW_DIRECT  11    // When held the above motion buttons do DIRECT TO SERIAL PORT

// Joystick controls from /joy array
#define AXIS_LINEAR       1
#define AXIS_ANGULAR      0

// Define scale factors where 1.0 is full scale joystick
#define AXIS_LINEAR_SCALE    10.0
#define AXIS_ANGULAR_SCALE   10.0


class JoyInput
{
public:
    JoyInput();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher  cmd_vel_pub_;
    ros::Subscriber joy_sub_;

};

 /* --------------------------------------------------------------------------------
 *  START: { Direct serial control code which will be obsolited by outbound topic messages
 *
 *  DEGRADATED CODE:  We will kill all this code but if you want to look at it, :set tabstop=4
 */

//  Helper routine for open of serial port
int openSerialPort(const char *spPortDev) {
	//const char  *spPortDev = SERIAL_DEV_TO_MOTOR_DRIVER; // if we want to hard code it, do this instead

	int   spFd;                                   // File descriptor for serial port

	spFd = open(spPortDev, O_RDWR | O_NOCTTY);   // open port for read/write but not as controlling terminal
	if (spFd < 0) {
		ROS_ERROR("%s: Error in open of serial port '%s' for servo controller! ", THIS_NODE_NAME, spPortDev);
		// we just return the bad file descriptor also as an error
	}
	return spFd;
}


/*
 * Reset drive controller with simple PiBot protocol
 *
 * At this time this does direct serial access to Arduino running the bridge firmware
 * Protocol:    r 
 */
int drive_resetCtrl()
{
	std::ostringstream	wheelDriveCmd;
	int	spFd = 0;
    int retCode = 0;

	// Open the serial port and blurt out the wheel drive command
	spFd = openSerialPort(SERIAL_DEV_TO_MOTOR_DRIVER);
	if (spFd < 0)  {
		ROS_ERROR("%s: Error in opening of serial port dev for motor control", THIS_NODE_NAME);
		return -3;
	}

    // Format the rather simple minded wheel control command
    wheelDriveCmd << "r" << "\r\n";


    ROS_INFO("Drive wheels with this control string: %s", wheelDriveCmd.str().c_str());
	int numChars = wheelDriveCmd.str().size();

    // Write all  bytes to the uart and thus the controller
	if ((write(spFd, wheelDriveCmd.str().c_str(), numChars)) != numChars) {   
			ROS_ERROR("%s: Error in initial writing to serial port for initServoHardware() ", THIS_NODE_NAME);
			retCode = -2;
	}

    close(spFd);	// be a good boy scout and leave the camp as you had found it

    return retCode;
}

/*
 * Set the right and left motor drive speeds using PiRobot protocol
 *
 * TODO:  Need to shoot off a command to ros arduino bridge.
 *
 * At this time this does direct serial access to Arduino running the bridge firmware
 * Protocol:    w <rightSpeed> <leftSpeed>
 */
int drive_SetWheelSpeeds(int rightSpeed, int leftSpeed)
{
	std::ostringstream	wheelDriveCmd;
	int	spFd = 0;
    int retCode = 0;

    // Optionally implement limit checks on speed once they are defined here
	if ((rightSpeed < 0) || (rightSpeed > WHEEL_SPEED_MAX)) {
		ROS_ERROR("%s: Illegal right wheel drive level of %d [max = %d]", THIS_NODE_NAME, rightSpeed, WHEEL_SPEED_MAX);
		return -1;
    }
	if ((leftSpeed < 0) || (leftSpeed > WHEEL_SPEED_MAX)) {
		ROS_ERROR("%s: Illegal left wheel drive level of %d [max = %d]", THIS_NODE_NAME, leftSpeed, WHEEL_SPEED_MAX);
		return -2;
    }
    
	// Open the serial port and blurt out the wheel drive command
	spFd = openSerialPort(SERIAL_DEV_TO_MOTOR_DRIVER);
	if (spFd < 0)  {
		ROS_ERROR("%s: Error in opening of serial port dev for motor control", THIS_NODE_NAME);
		return -3;
	}

    // Format the rather simple minded wheel control command
    wheelDriveCmd << "m " << rightSpeed << " " << leftSpeed  << "\r\n";


    ROS_INFO("Drive wheels with this control string: %s", wheelDriveCmd.str().c_str());
	int numChars = wheelDriveCmd.str().size();

    // Write all  bytes to the uart and thus the controller
	if ((write(spFd, wheelDriveCmd.str().c_str(), numChars)) != numChars) {  
			ROS_ERROR("%s: Error in initial writing to serial port for initServoHardware() ", THIS_NODE_NAME);
			retCode = -2;
	}

    close(spFd);	// be a good boy scout and leave the camp as you had found it

    return retCode;
}


/* 
 *  END: } Direct serial control code which will be obsolited by outbound topic messages
 *  -------------------------------------------------------------------------------- */


JoyInput::JoyInput():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  /* Advertise our topic we choose if we want to generate any output
   * vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
   */

  // Subscribe to the /joy topic for input from joystick
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyInput::joyCallback, this);

  // Publish on the twist topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

}

void JoyInput::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    int button = -1;

    // Initializes with zeros by default.
    geometry_msgs::Twist cmd_vel_msg;

    // Get option from switch button if we are doing DIRECT hw control, NOT through bridge
    bool direct_serial_cmd_vel = false;   // Default is to use ros twist topic
    if (joy->buttons.size() >= BUTTON_HW_DIRECT) {
        direct_serial_cmd_vel = joy->buttons[BUTTON_HW_DIRECT];   // Use direct serial, not arduino bridge
    }
    /*
     * Twist topic output on joystick with button press
     */
    bool enable_joy_cmd_vel = true;   // Default is always enable joystick to cmd_vel twist output
    if(joy->buttons.size() >= BUTTON_CMD_VEL) {
        enable_joy_cmd_vel = joy->buttons[BUTTON_CMD_VEL];
    }

    if (enable_joy_cmd_vel) {
        // Pull the joysticks for forward and turning off of /joy topic
        cmd_vel_msg.linear.x = joy->axes[AXIS_LINEAR] * AXIS_LINEAR_SCALE;
        cmd_vel_msg.angular.z = joy->axes[AXIS_ANGULAR] * AXIS_ANGULAR_SCALE;

        // Publish classic 'twist' velocities to the rest of the system
        cmd_vel_pub_.publish(cmd_vel_msg);

    } else {
        // If the enable button is not active, send an all stop twist message
        cmd_vel_pub_.publish(cmd_vel_msg);
    }

    /*
     * Direct actions on buttons
     */
    for(size_t i=0;i<joy->buttons.size();i++) {
        if (joy->buttons[i] != 0) {    // This button is being held down now
            button = i;
            switch(button) {
            case BUTTON_CMD_VEL:    // Do nothing, this is done for joystick code
            case BUTTON_HW_DIRECT:  // Do nothing, this is for direct hardware serial control
                break;
            case BUTTON_FORWARD:
                ROS_INFO("Command to start forward drive mode.");
                if (direct_serial_cmd_vel) {
                    drive_SetWheelSpeeds(10,10);
                } else {
                    cmd_vel_msg.linear.x = 7;
                    cmd_vel_msg.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }

                break;
            case BUTTON_RIGHT:
                ROS_INFO("Command to start right   drive mode.");
                if (direct_serial_cmd_vel) {
                    drive_SetWheelSpeeds(2,8);
                } else {
                    cmd_vel_msg.linear.x = 7;
                    cmd_vel_msg.angular.z = 5;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }
                break;
            case BUTTON_LEFT:
                ROS_INFO("Command to start left    drive mode.");
                if (direct_serial_cmd_vel) {
                    drive_SetWheelSpeeds(8,2);
                } else {
                    cmd_vel_msg.linear.x = 7;
                    cmd_vel_msg.angular.z = -5;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }
                break;
            case BUTTON_STOP:
                ROS_INFO("Command to STOP [%d] any active driving.",button);
                if (direct_serial_cmd_vel) {
                    drive_SetWheelSpeeds(0,0);
                } else {
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }
                break;
            case BUTTON_RESET:
                ROS_INFO("Reset controller.");
                drive_resetCtrl();
                break;

            default:
                ROS_INFO("Got joystick input with unrecognized button %2d as 1st button down [%d buttons %d axes]",
                    button, joy->buttons.size(), joy->axes.size());
                break;
            }
        }
    }

    if (button < 0) {
        ROS_DEBUG("Got joystick input with no button pressed            [%d buttons %d axes]",
            joy->buttons.size(), joy->axes.size());
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_input");
    JoyInput joy_input;

    ros::spin();
}
