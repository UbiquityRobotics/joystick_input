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

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <fcntl.h>
#include <iterator>
#include <list>
#include <queue>

#include <sensor_msgs/Joy.h>           // joy topic message format
#include <geometry_msgs/Twist.h>       // classic 'twist' messages for bot motion commands

// Things used to tell the bot nav stack to move to a specific location
#include <move_base_msgs/MoveBaseAction.h>	// Messages to direct nav stack movement
#include <actionlib/client/simple_action_client.h>
typedef  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define THIS_NODE_NAME	"joy_input"    // Node name hard coded
#define LOOPS_PER_SEC   10

// Locate these in a system-wide include which I am not aware of at this time
#define  SERIAL_DEV_TO_MOTOR_DRIVER   "/dev/ttyAMA0"    // Raspberry Pi main serial port
#define  WHEEL_SPEED_MAX          20                // max wheel speed
#define  DEFAULT_SPEED            10
#define  DEFAULT_TURNING          5

// Button definitions for the array place used for a given controller
// use rostopic echo /joy for your controller and define buttons below

#define JOYSTICK_XBOX              0
#define JOYSTICK_PS3               1

// XBOX 360 buttons (in terms of the wireless controller)
#define BUTTON_XBOX_L_TOP_FRONT    4     // top front left hand button
#define BUTTON_XBOX_R_TOP_FRONT    5     // top front right hand button
#define BUTTON_XBOX_BACK           6     // Little round 'Back' button to left of center big 'X' round button
#define BUTTON_XBOX_START          7     // Little round 'Start' button to right of center big 'X' round button

#define BUTTON_XBOX_YELLOW_Y       3     // Yellow 'Y' button 
#define BUTTON_XBOX_RED_B          1     // Red 'B' button
#define BUTTON_XBOX_BLUE_X         2     // Blue 'X' button
#define BUTTON_XBOX_GREEN_A        0     // Green 'A' button

#define BUTTON_XBOX_RESET          999   // Not implemented


// PS3 DualShock3 buttons (in terms of the wireless controller)
#define BUTTON_PS3__L_TOP_FRONT    4     // top front left hand button
#define BUTTON_PS3__R_TOP_FRONT    5     // top front right hand button
#define BUTTON_PS3__SELECT         6     // Select button in center section on top
#define BUTTON_PS3__START          7     // Start button in center section on top

#define BUTTON_PS3__GRN_TRIANGLE   3     // Green triangle key to right
#define BUTTON_PS3__RED_CIRCLE     1     // Yellow 'Y' button 
#define BUTTON_PS3__BLUE_X         0     // Blue X button
#define BUTTON_PS3__VIO_SQUARE     2     // Violet square button


// We have a button map so we can support different joystick types
typedef struct buttonMap_t {
    int buttonCmdVel;              // When held down the left joystick goes to /cmd_vel topic
    int buttonHwDirect;            // When held down we use colored keys for direct serial HW commands
    int buttonNavGotoTarget;       // When held down we go to NAV point if colored key is hit

    //  The 4 buttons on right have 3 functions each depending on the control buttons held or not
    int buttonForwardNav1;         // Move forward (direct or cmd_vel) or goto Nav1 
    int buttonRightNav2;           // Move Right   (direct or cmd_vel) or goto Nav2
    int buttonLeftNav3;            // Move Left    (direct or cmd_vel) or goto Nav4
    int buttonStopNav4;            // Move Stop    (direct or cmd_vel) or goto Nav3

    int buttonReset;
} ButtonMap;

// if we need global:   ButtonMap g_buttonMap;

// Joystick controls from /joy array for XBOX 360
#define AXIS_XBOX_LEFT_FWD_BACK    1     // Left joystick front to back
#define AXIS_XBOX_LEFT_SIDE_SIDE   0     // Left joystick side to side

// Joystick controls from /joy array for XBOX 360
#define AXIS_PS3__LEFT_FWD_BACK    1
#define AXIS_PS3__LEFT_SIDE_SIDE   0

// We have a Axis map so we can support different joystick types
typedef struct axisMap_t {
    int axisLinear;
    int axisAngular;
} AxisMap;

// Define scale factors where 1.0 is full scale joystick
#define AXIS_LINEAR_SCALE    10.0
#define AXIS_ANGULAR_SCALE   10.0

// A poor mans state for this node
typedef struct NodeState {
    bool   disableNavStack;    // If set non-zero we disable nav stack code

    double cmdVelSpeed;        // The velocity used for simple button twist velocity
    double cmdVelTurnSpeed;    // The velocity used for simple button twist turning 

    double cmdVelJoyMax;       // The scaling factor used forward/back joystick gain
    double cmdVelJoyTurnMax;   // The scaling factor used forward/back joystick gain

    double directHwSpeed;      // The velocity used for simple button serial speed 
    double directHwTurnSpeed;  // The velocity used for simple button serial turning


    // These are target destinations that come in as ros params.
    // These will go bye-bye for cleaner approach after big hoopla show in mid May 2015
    double target1_x;
    double target1_y;
    double target1_w;

    double target2_x;
    double target2_y;
    double target2_w;

    double target3_x;
    double target3_y;
    double target3_w;

    double target4_x;
    double target4_y;
    double target4_w;
} NodeState;


/*
 * Utility routines
 */
// Helper to fetch current ROS int parameter and indicate if there was a change.  
// Return true if it does not match default
bool fetchIntRosParam(ros::NodeHandle &nh, std::string paramName, int &value) 
{
  int paramInt;
  bool retCode = false;

  if (nh.getParam(paramName, paramInt)) {
    if (value != paramInt) {	// If it has changed do a log
      ROS_INFO("ROS Parameter '%s' changed from %d to %d.", paramName.c_str(), value, paramInt);
      value = paramInt;		// If we found it in ROS parameter server use this value
      retCode = true;
    }
  }
  return retCode;
}

// Helper to fetch current ROS int parameter and indicate if there was a change.  
// Return true if it does not match default
//
// BROKEN AND I AM UNABLE TO FIGURE THIS THING OUT!!!
// Maybe a DEEP look into:  http://wiki.ros.org/roscpp/Overview/Parameter%20Server
//
bool fetchFloatRosParam(ros::NodeHandle &nh, std::string paramName, float &value) 
{
  double paramDouble;
  float  paramFloat;
  std::string valueStr;
  std::string::size_type sz;
  bool retCode = false;

  if (nh.getParam(paramName, paramDouble)) {
    paramFloat = (float)(paramDouble);
    if (value != paramFloat) {	// If it has changed do a log
      ROS_INFO("ROS Parameter '%s' changed from %f to %f.", paramName.c_str(), value, paramFloat);
      value = paramFloat;	// If we found it in ROS parameter server use this value
      retCode = true;
    }
  }
  return retCode;
}


// Routine to read or re-read bot rosparams from ROS param server or defaults
void  refreshBotStateParams(ros::NodeHandle &nh, NodeState &state) {
  int   paramInt;
  float paramFloat;
  std::string paramStr;

  ROS_DEBUG("%s: Refreshing ROS Parameters", THIS_NODE_NAME);

  paramInt = state.disableNavStack;
  if (fetchIntRosParam(nh, "/joy_input/disable_nav_stack", paramInt)) {
    // sorry this is sort of 'backwards' as we use default 0 as our default
    if (paramInt == 0) {
      ROS_INFO("%s: Navigation stack will be enabled. ", THIS_NODE_NAME);
      state.disableNavStack = 0;
    } else {
      ROS_INFO("%s: Navigation stack will be DISABLED. ", THIS_NODE_NAME);
      state.disableNavStack = 1;
    }
  }

  paramFloat = state.cmdVelSpeed;
  if (fetchFloatRosParam(nh, "/joy_input/cmd_vel_speed", paramFloat)) {
    state.cmdVelSpeed = paramFloat;
  }
  paramFloat = state.cmdVelTurnSpeed;
  if (fetchFloatRosParam(nh, "/joy_input/cmd_vel_turn_speed", paramFloat)) {
    state.cmdVelTurnSpeed = paramFloat;
  }

  paramFloat = state.cmdVelJoyMax;
  if (fetchFloatRosParam(nh, "/joy_input/cmd_vel_joy_max", paramFloat)) {
    state.cmdVelJoyMax = paramFloat;
  }
  paramFloat = state.cmdVelJoyTurnMax;
  if (fetchFloatRosParam(nh, "/joy_input/cmd_vel_joy_turn_max", paramFloat)) {
    state.cmdVelJoyTurnMax = paramFloat;
  }

  paramFloat = state.directHwSpeed;
  if (fetchFloatRosParam(nh, "/joy_input/direct_hw_speed", paramFloat)) {
    state.directHwSpeed = paramFloat;
  }
  paramFloat = state.directHwTurnSpeed;
  if (fetchFloatRosParam(nh, "/joy_input/direct_hw_turn_speed", paramFloat)) {
    state.directHwTurnSpeed = paramFloat;
  }



  // This is some really dirty 'just do it' code.  PLEASE don't show this to my mother ...  ;-)
  paramFloat = state.target1_x;
  if (fetchFloatRosParam(nh, "/joy_input/target_1_x", paramFloat)) {
    state.target1_x = paramFloat;
  }
  paramFloat = state.target1_y;
  if (fetchFloatRosParam(nh, "/joy_input/target_1_y", paramFloat)) {
    state.target1_y = paramFloat;
  }
  paramFloat = state.target1_w;
  if (fetchFloatRosParam(nh, "/joy_input/target_1_w", paramFloat)) {
    state.target1_w = paramFloat;
  }

  paramFloat = state.target2_x;
  if (fetchFloatRosParam(nh, "/joy_input/target_2_x", paramFloat)) {
    state.target2_x = paramFloat;
  }
  paramFloat = state.target2_y;
  if (fetchFloatRosParam(nh, "/joy_input/target_2_y", paramFloat)) {
    state.target2_y = paramFloat;
  }
  paramFloat = state.target2_w;
  if (fetchFloatRosParam(nh, "/joy_input/target_2_w", paramFloat)) {
    state.target2_w = paramFloat;
  }

  paramFloat = state.target3_x;
  if (fetchFloatRosParam(nh, "/joy_input/target_3_x", paramFloat)) {
    state.target3_x = paramFloat;
  }
  paramFloat = state.target3_y;
  if (fetchFloatRosParam(nh, "/joy_input/target_3_y", paramFloat)) {
    state.target3_y = paramFloat;
  }
  paramFloat = state.target3_w;
  if (fetchFloatRosParam(nh, "/joy_input/target_3_w", paramFloat)) {
    state.target3_w = paramFloat;
  }

  paramFloat = state.target4_x;
  if (fetchFloatRosParam(nh, "/joy_input/target_4_x", paramFloat)) {
    state.target4_x = paramFloat;
  }
  paramFloat = state.target4_y;
  if (fetchFloatRosParam(nh, "/joy_input/target_4_y", paramFloat)) {
    state.target4_y = paramFloat;
  }
  paramFloat = state.target4_w;
  if (fetchFloatRosParam(nh, "/joy_input/target_4_w", paramFloat)) {
    state.target4_w = paramFloat;
  }

}

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
 *  -------------------------------------------------------------------------------- 
 */

// MoveGoals are things we want to do next in terms of movement
// The goals can have pre-requisites.  We cannot goto an object unless we are can see it.
// The goal types below have to be contained in a BotGoal context or they don't have much meaning
// Some Goals are direct commands and some require logic such as search for an object.
enum MoveGoalType {
        G_NONE,                 // Used if a goal is not required cases such as we finished last goal
        G_CLEAR_GOALS,          // Clear the list of goals (as sort of reset of next goals
        G_MOVE_TO_MAP_LOCATION, // Move to some specific location on the map
        G_MOVE_A_DISTANCE       // Move for a given distance in x and y
};


// MoveGoal:  Define a class to hold a movement goal and context 
class MoveGoal {
    private:
    MoveGoalType type;    // The goal identifier or type
    std::string desc;     // String that can be used to describe this goal
    float    x;          // X Coordinate this goal refers to
    float    y;          // Y Coordinate this goal refers to
    float    w;          // Orientation
    float    z;          // An X Coordinate this goal refers to
    float    speed;      // A speed we may wish to specify
    ros::Time startTime;  // Time we started this goal in system clock units
    float    duration;   // Time that can be used if the goal is for some number of seconds
    float    timeout;    // A timeout value in seconds so we can decided if we no longer care

   public:
    // These constructors make no assumptions on value initializations so are a bit verbose
    // Not expected to be used with all defaults
    MoveGoal() :
        type(G_NONE), desc("Goal"), x(0.0), y(0.0), z(0.0), w(0.0),
        speed(0.0), startTime(ros::Time::now()), duration(0.0), timeout(0.0) {}

    MoveGoal(MoveGoalType t, float xpos, float ypos, float wpos) :
        type(t), desc("Goal"), x(xpos), y(ypos), z(0.0), w(wpos),
        speed(0.0), startTime(ros::Time::now()), duration(0.0), timeout(0.0) {}

    MoveGoal(MoveGoalType t, std::string dstr, float xpos, float ypos, float wpos) :
        type(t), desc(dstr), x(xpos), y(ypos), z(0.0), w(wpos),
        speed(0.0), startTime(ros::Time::now()), duration(0.0), timeout(0.0) {}

    // getters
    MoveGoalType getType() { return type; };
    std::string  getTypeName() { 
        std::string goalName;
        switch (type) {
            case G_NONE:                 goalName = "No Goal";               break;
            case G_CLEAR_GOALS:          goalName = "Clear goals";           break;
            case G_MOVE_TO_MAP_LOCATION: goalName = "Move to map location";  break;
            case G_MOVE_A_DISTANCE:      goalName = "Move for a distance";   break;
            default:                     goalName = "Undefined goal";        break;
        }
        return goalName;
    };

    float getX()        { return x; };
    float getY()        { return y; };
    float getZ()        { return z; };
    float getW()        { return w; };
    float getSpeed()    { return speed; };
    float getDuration() { return duration; };
    std::string  getDescription() { return desc; }
};


class JoyInput
{
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    std::queue<MoveGoal> moveGoalQueue;

    ros::NodeHandle nh_;

    int joystickType;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher  cmd_vel_pub_;
    ros::Subscriber joy_sub_;

public:
    JoyInput();

    NodeState nodeState;

    ButtonMap buttonMap;
    AxisMap   axisMap;

    // Here we can push a goal or pull a goal
    int numMoveGoals() {
        int numGoals = (int)moveGoalQueue.size();
        return numGoals;
    }

    int pushMoveGoal( MoveGoal moveGoal) {
        moveGoalQueue.push(moveGoal);
    }

    // Pop off a goal but if there are none, return 0
    int popMoveGoal(MoveGoal &moveGoal) {
        if (moveGoalQueue.empty()) {
            return 0;
        }
        moveGoal = moveGoalQueue.front();
        moveGoalQueue.pop();
        return 1;
    }

    // Set joystick type and return 0 for ok, other for fault
    int setJoystickType(int type) {
        switch (type) {
            case JOYSTICK_XBOX:
                ROS_INFO("%s: Joystick type set to XBOX",THIS_NODE_NAME);
                joystickType = type;
                break;
            case JOYSTICK_PS3:
                ROS_ERROR("%s: Joystick type set to PS3",THIS_NODE_NAME);
                joystickType = type;
                break;
        
            default:
                return -1;
            break;
        }

        joystickConfig(type);	// Setup the actual button and axis values
        return 0;
    }

    // We hope to support different joystick types and this allows functions to be mapped
    int joystickConfig(int type) {
        int retCode = -1;

        switch (type) {
            case JOYSTICK_XBOX:
                joystickType = type;
                retCode = 0;

                buttonMap.buttonCmdVel        = BUTTON_XBOX_L_TOP_FRONT;  // Hold down to allow left joystick to /cmd_vel
                buttonMap.buttonHwDirect      = BUTTON_XBOX_START;        // Hold down to use direct serial motor control
                buttonMap.buttonNavGotoTarget = BUTTON_XBOX_BACK;         // Hold down to 'shift' to nav goto target

                buttonMap.buttonForwardNav1   = BUTTON_XBOX_YELLOW_Y;
                buttonMap.buttonRightNav2     = BUTTON_XBOX_RED_B;
                buttonMap.buttonLeftNav3      = BUTTON_XBOX_BLUE_X;
                buttonMap.buttonStopNav4      = BUTTON_XBOX_GREEN_A;

                buttonMap.buttonReset         = BUTTON_XBOX_RESET;

                axisMap.axisLinear            = AXIS_XBOX_LEFT_FWD_BACK;
                axisMap.axisAngular           = AXIS_XBOX_LEFT_SIDE_SIDE;;
                break;

            case JOYSTICK_PS3:
            default:
                break;
        }
    }

};


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

    // Get option from button if we are doing DIRECT hw control with the 4 colored keys
    bool direct_serial_cmd_vel_mode = false;   // Default is to use ros twist topic
    if (joy->buttons.size() >= buttonMap.buttonHwDirect) {
        direct_serial_cmd_vel_mode = joy->buttons[buttonMap.buttonHwDirect];   // Use direct serial, not arduino bridge
    }

    // Get option from button if we are doing navigation goto with the 4 colored keys
    bool nav_goto_target_mode = false;   // Default is to use the keys for manual bot control
    if (joy->buttons.size() >= buttonMap.buttonNavGotoTarget) {
        nav_goto_target_mode = joy->buttons[buttonMap.buttonNavGotoTarget];  // Colored keys will goto nav point
    }

    /*
     * Twist topic output on joystick with button press
     */
    bool enable_joy_cmd_vel = true;   // Default is always enable joystick to cmd_vel twist output
    if(joy->buttons.size() >= buttonMap.buttonCmdVel) {
        enable_joy_cmd_vel = joy->buttons[buttonMap.buttonCmdVel];
    }

    if (enable_joy_cmd_vel) {
        // Pull the joysticks for forward and turning off of /joy topic
        cmd_vel_msg.linear.x = joy->axes[axisMap.axisLinear] * nodeState.cmdVelJoyMax;
        cmd_vel_msg.angular.z = joy->axes[axisMap.axisAngular] * nodeState.cmdVelJoyTurnMax;

        // Publish classic 'twist' velocities to the rest of the system
        cmd_vel_pub_.publish(cmd_vel_msg);

    } else {
        // If the enable button is not active, send an all stop twist message
        cmd_vel_pub_.publish(cmd_vel_msg);
    }

    /*
     * Direct actions on buttons
     */
    std::string controlMode;
    controlMode = (direct_serial_cmd_vel_mode) ? "direct serial control" : "/cmd_vel topic twist message";
    int multiControlBits = 0;

    
    for(size_t i=0;i<joy->buttons.size();i++) {
        if (joy->buttons[i] != 0) {    // This button is being held down now
            button = i;

            // We do big ol if then else here because switch only supports case constants
            // TODO!!!  Logic for control buttons will get confused if multiple control keys at once
            if ((button == buttonMap.buttonCmdVel)        ||
                (button == buttonMap.buttonNavGotoTarget) ||
                (button == buttonMap.buttonHwDirect)) { 
                // form a composite control key code although we may never use it
                // Note that we cannot use this in other parts of the combo-if because
                // depending on button number not all of the bits may be set yet
                multiControlBits |= 1 << button;

            } else if (button == buttonMap.buttonForwardNav1) {
                if (direct_serial_cmd_vel_mode) {
                    ROS_INFO("Command to start forward drive mode using %s", controlMode.c_str());
                    drive_SetWheelSpeeds((int)nodeState.directHwSpeed,(int)nodeState.directHwSpeed);
                } else if (nav_goto_target_mode) {
                    ROS_INFO("Command to Navigate location 1");
                    pushMoveGoal(MoveGoal(G_MOVE_TO_MAP_LOCATION, "Move to location 1", 
                        nodeState.target1_x, nodeState.target1_y, nodeState.target1_w ));
                } else {
                    ROS_INFO("Command to start forward drive mode using %s", controlMode.c_str());
                    cmd_vel_msg.linear.x = nodeState.cmdVelSpeed;
                    cmd_vel_msg.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }

            } else if (button == buttonMap.buttonRightNav2) {
                if (direct_serial_cmd_vel_mode) {
                    ROS_INFO("Command to start right   drive mode using %s", controlMode.c_str());
                    drive_SetWheelSpeeds((int)nodeState.directHwTurnSpeed,(int)nodeState.directHwSpeed);
                } else if (nav_goto_target_mode) {
                    ROS_INFO("Command to Navigate location 2");
                    pushMoveGoal(MoveGoal(G_MOVE_TO_MAP_LOCATION, "Move to location 1", 
                        nodeState.target2_x, nodeState.target2_y, nodeState.target2_w));
                } else {
                    ROS_INFO("Command to start right   drive mode using %s", controlMode.c_str());
                    cmd_vel_msg.linear.x = nodeState.cmdVelSpeed;
                    cmd_vel_msg.angular.z = nodeState.cmdVelTurnSpeed;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }

            } else if (button == buttonMap.buttonLeftNav3) {
                if (direct_serial_cmd_vel_mode) {
                    ROS_INFO("Command to start left    drive mode using %s", controlMode.c_str());
                    drive_SetWheelSpeeds((int)nodeState.directHwSpeed,(int)nodeState.directHwTurnSpeed);
                } else if (nav_goto_target_mode) {
                    ROS_INFO("Command to Navigate location 3");
                    pushMoveGoal(MoveGoal(G_MOVE_TO_MAP_LOCATION, "Move to location 1", 
                        nodeState.target3_x, nodeState.target3_y, nodeState.target3_w));
                } else {
                    ROS_INFO("Command to start left    drive mode using %s", controlMode.c_str());
                    cmd_vel_msg.linear.x = nodeState.cmdVelSpeed;
                    cmd_vel_msg.angular.z = (double)(-1.0) * nodeState.cmdVelTurnSpeed;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }

            } else if (button == buttonMap.buttonStopNav4) {
                if (direct_serial_cmd_vel_mode) {
                    ROS_INFO("Command to STOP any active driving  using %s", controlMode.c_str());
                    drive_SetWheelSpeeds(0,0);
                } else if (nav_goto_target_mode) {
                    ROS_INFO("Command to Navigate location 4");
                    pushMoveGoal(MoveGoal(G_MOVE_TO_MAP_LOCATION, "Move to location 1", 
                        nodeState.target4_x, nodeState.target4_y, nodeState.target4_w));
                } else {
                    ROS_INFO("Command to STOP any active driving  using %s", controlMode.c_str());
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel_msg); // Publish classic 'twist' velocities 
                }

            } else if (button == buttonMap.buttonReset) {
                // TODO: Reset ctrl Not implemented. Perhaps best if we do reset with multiple button code?
                ROS_INFO("Reset controller.");
                drive_resetCtrl();

            } else {
                ROS_INFO("Got joystick input with unrecognized button %2d as 1st button down [%d buttons %d axes]",
                    button, joy->buttons.size(), joy->axes.size());
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

    ros::NodeHandle nh;

    // Define our main loop rate as a ROS rate and we will let ROS do waits between loops
    ros::Rate loop_rate(LOOPS_PER_SEC);

    int joyType = JOYSTICK_XBOX;
    // if (fetchIntRosParam(nh, "/joy_input/joystick_type", joyType)) {
    // }
    if (joy_input.setJoystickType(joyType) < 0) {
        ROS_ERROR("%s: Illegal joystick type of %d. XBOX=%d PS3=%d",THIS_NODE_NAME,
            joyType, JOYSTICK_XBOX, JOYSTICK_PS3);
    } else {
        ROS_INFO("%s: Joystick type is set to %d. XBOX=%d PS3=%d",THIS_NODE_NAME,
              joyType, JOYSTICK_XBOX, JOYSTICK_PS3);
    }

    // Set a few default params in case not preset as ros params
    joy_input.nodeState.cmdVelSpeed         = (double)(DEFAULT_SPEED);
    joy_input.nodeState.cmdVelTurnSpeed     = (double)(DEFAULT_TURNING);
    joy_input.nodeState.cmdVelJoyMax        = AXIS_LINEAR_SCALE;
    joy_input.nodeState.cmdVelJoyTurnMax    = AXIS_ANGULAR_SCALE;
    joy_input.nodeState.directHwSpeed       = (double)(DEFAULT_SPEED);
    joy_input.nodeState.directHwTurnSpeed   = (double)(DEFAULT_TURNING);


    // We are going to refresh parameters from time to time so those go in a refresh utility
    refreshBotStateParams(nh, joy_input.nodeState);


    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
  
    if (joy_input.nodeState.disableNavStack == 0) {
        //wait for the action server to come up
        ROS_INFO("%s: Wait for the move_base action server to come up ...", THIS_NODE_NAME);
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_DEBUG("Waiting for the move_base action server to come up");
        }
        ROS_INFO("The move_base action server is active.");
    }
  
    move_base_msgs::MoveBaseGoal mbGoal;
    mbGoal.target_pose.header.frame_id = "base_link";

    int loopCount = 0;
    while (ros::ok())
    {
        if ((loopCount % (LOOPS_PER_SEC*5))== 1) { // pull in any changed parameters via ros param server
            refreshBotStateParams(nh, joy_input.nodeState);
        }

        MoveGoal goal;
        if (joy_input.numMoveGoals() > 0)  {
           joy_input.popMoveGoal(goal);
           ROS_DEBUG("%s: Processing new goal of type %s with description '%s' with X= %f Y= %f Z= %f W= %f", 
               THIS_NODE_NAME, goal.getTypeName().c_str(), goal.getDescription().c_str(), 
               goal.getX(), goal.getY(), goal.getZ(), goal.getW());

           switch (goal.getType()) {
               case  G_MOVE_TO_MAP_LOCATION:    // Move to some specific location on the map

                   // TODO!!! Put this messy sending stuff in a helper !!!
                   mbGoal.target_pose.header.stamp = ros::Time::now();
                   mbGoal.target_pose.pose.position.x = goal.getX();
                   mbGoal.target_pose.pose.position.y = goal.getY();
                   mbGoal.target_pose.pose.orientation.w = goal.getW();

                   if (joy_input.nodeState.disableNavStack == 0) {
                       ROS_DEBUG("%s: Sending new MoveBaseGoal with user desc '%s' and X= %f Y= %f W= %f", 
                           THIS_NODE_NAME, goal.getDescription().c_str(), goal.getX(), goal.getY(), goal.getW());

                       ac.sendGoal(mbGoal);

                       ac.waitForResult();   // TODO: We really should be smarter than blind wait

                       if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                           ROS_INFO("%s: MoveBase with user desc '%s' to X= %f Y= %f W= %f SUCCEEDED!", 
                               THIS_NODE_NAME, goal.getDescription().c_str(), goal.getX(), goal.getY(), goal.getW());
                       } else {
                           ROS_ERROR("%s: MoveBase with user desc '%s' to X= %f Y= %f W= %f FAILED!", 
                               THIS_NODE_NAME, goal.getDescription().c_str(), goal.getX(), goal.getY(), goal.getW());
                       }
                   } else {
                       ROS_ERROR("%s: MoveBase DISABLED but we got goal with desc '%s' to X= %f Y= %f W= %f", 
                           THIS_NODE_NAME, goal.getDescription().c_str(), goal.getX(), goal.getY(), goal.getW());
                   }
                   break;
               default:
                   ROS_INFO("%s: Unrecognized goal type %d with description '%s' with X= %f Y= %f ", THIS_NODE_NAME,
                       goal.getType(), goal.getDescription().c_str(), goal.getX(), goal.getY());
                   break;
           }
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++loopCount;   
        if (loopCount > 30000) { loopCount = 1; }
    }
}
