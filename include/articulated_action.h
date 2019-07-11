#ifndef ARTICULATEDACTION_H
#define ARTICULATEDACTION_H

#include "ros/ros.h"
#include "articulated/StepperDriver.h"
#include "articulated/serial_msg.h"
#include "mechanism_calculations.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <sstream>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <articulated/ikAction.h>
#include <articulated/calibrateAction.h>

//Calibrate Key Listener Stuff
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_TAB 0x09


class ArticulatedAction
{
public:
	ArticulatedAction(std::string ik_server_name, std::string cal_server_name);
	~ArticulatedAction();
	
private:
	//FUNCTIONS
	void setSteppers(std_msgs::Float64MultiArray goal);
	void serialCallback(articulated::serial_msg data);
	int calcSteps(double angle, double stepper_id);
	void ikCB(const articulated::ikGoalConstPtr &goal);
	void ikFB();
	void executeCalCB(const articulated::calibrateGoalConstPtr &goal);




	//PUBLISHERS / SUBSCRIBERS
	ros::Publisher joint_state_pub_;
	ros::Publisher serial_pub_;
	ros::Subscriber serial_sub_;
  	
  	//OBJECTS
	ros::NodeHandle nh_;
	MechCalc mech_;
  	
	//VARS
	double stepper_angle_current_[3];
	bool stepper_state_[3];
	geometry_msgs::Pose ee_pose_;

  	//IKACTION
  	actionlib::SimpleActionServer<articulated::ikAction> ik_as_; //be sure to declare after node handle
	std::string ik_action_name_;
	articulated::ikFeedback ik_feedback_; //create messages that are used to published feedback/result
	articulated::ikResult ik_result_;
	geometry_msgs::Pose ik_goal_;
	geometry_msgs::Pose ik_error_;

	//CABLIBRATE ACTION
	actionlib::SimpleActionServer<articulated::calibrateAction> cal_as_;
};

#endif