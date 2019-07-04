#include "articulated_action.h"

ArticulatedAction::ArticulatedAction(std::string ik_server_name) :
	ik_as_(nh_, ik_server_name, false),
	ik_action_name_(ik_server_name) 

	//boost::bind(&ArticulatedAction::ikCB, this, _1)
{
	//Define PUBLISHERS
  	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("articulated/joint_state", 1000);
  	serial_pub_ = nh_.advertise<articulated::serial_msg>("articulated/serial/send", 1000);
	//Define SUBSCRIBERS
	serial_sub_ = nh_.subscribe("articulated/serial/receive", 1000, &ArticulatedAction::serialCallback, this);

	ik_as_.start();

	//SET INITIAL JOINT ANGLES
	//NOTE: This should be replaced with a calibration procedure
	stepper_angle_current_[0] = 0;
  	stepper_angle_current_[1] = M_PI/2;
  	stepper_angle_current_[2] = 0;
  	//I DONT CARE ABOUT THE RETURNED GEO MSG, NEED TO GIVE MECH THE STARTING STEP ANGLES
  	mech_.getEEPose(stepper_angle_current_);

  	
  	
	while (ros::ok())
	{
		//Publish Joint State
		joint_state_pub_.publish(mech_.joint_state_);
		if (ik_as_.isActive()) //Check if IK is currently processing goal
		{
			ikFB();
		}

		else if (ik_as_.isNewGoalAvailable()) //Check if new goal available
		{
			ROS_ERROR_STREAM("NEW GOAL");
			ikCB(ik_as_.acceptNewGoal()); //Accept / Send goal to the IK Callback

		}
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
}

ArticulatedAction::~ArticulatedAction()
{

}

void ArticulatedAction::ikCB(const articulated::ikGoalConstPtr &goal)
{
	double * jt_st_goal;
	bool flag;
	ik_goal_.position.x = goal->x;
	ik_goal_.position.y = goal->y;
	ik_goal_.position.z = goal->z;
	//CHECK IF GOAL IS LEGIT HERE. IF SO RUN INVERSE KINEMATICS
	std::tie(jt_st_goal, flag) = mech_.inverseKinematics(ik_goal_, stepper_angle_current_);
	if (!flag) //If Flag is high inverse kinematics failed
	{
		std_msgs::Float64MultiArray goal_msg_out;
		goal_msg_out.data.resize(3);
		goal_msg_out.data[0] = jt_st_goal[0];
		goal_msg_out.data[1] = jt_st_goal[1];
		goal_msg_out.data[2] = jt_st_goal[2];

		setSteppers(goal_msg_out);
	}
	else ROS_ERROR_STREAM("IK SOLUTION IS INACCURATE OR DOES NOT EXIST");
}

void ArticulatedAction::ikFB()
{
	ik_feedback_.x = ee_pose_.position.x; 
	ik_feedback_.y = ee_pose_.position.y; 
	ik_feedback_.z = ee_pose_.position.z; 
	ik_as_.publishFeedback(ik_feedback_);

	if (fabs(ee_pose_.position.x - ik_goal_.position.x) < 0.01)
	{
		if (fabs(ee_pose_.position.x - ik_goal_.position.x) < 0.01)
		{
			if (fabs(ee_pose_.position.x - ik_goal_.position.x) < 0.01)
			{	
				ik_result_.x = ik_feedback_.x;
				ik_result_.y = ik_feedback_.y;
				ik_result_.z = ik_feedback_.z;
				ik_as_.setSucceeded(ik_result_);
				ROS_ERROR_STREAM("IK SUCCESS");
			}
		}
	}
}
void ArticulatedAction::setSteppers(std_msgs::Float64MultiArray goal)
{
	for (int i = 0; i<3; i++)
	{
		if (fabs(goal.data[i] - stepper_angle_current_[i]) > M_PI/180)
		{
		  double g = goal.data[i] - stepper_angle_current_[i];
		  //Publish Step Goal in this funtion for the time being
		  
		  //spr is steps per revolution, the spr for each stepper is a param
		  //double spr;
		  //int g_steps = calcSteps(g, i);

		  std::stringstream g_string;
		  g_string << g;
		  //SET STEP POS arduino topic now takes goal as dq (in radians)
		  articulated::serial_msg g_msg;
		  g_msg.micro_id = i;
		  g_msg.topic = "set_step_pos";
		  g_msg.msg = g_string.str();
		  serial_pub_.publish(g_msg);
		  //ROS_ERROR_STREAM("sending stepper " << i << " goal of " << g_steps <<" steps");
		}
	}
}

void ArticulatedAction::serialCallback(articulated::serial_msg data)
{
	//FEEDBACK FROM SERIAL PORT, get stepper positions

	//Save incoming data... ID and Steps
	int i = data.micro_id;
	std::stringstream string_id; 
	string_id << i;

	double dq;
	std::stringstream string_dq(data.msg);
	string_dq >> dq;
	stepper_angle_current_[i] = stepper_angle_current_[i] + dq;

	//CREATE NEW FUNCTION To UPDATE MECH
	geometry_msgs::Pose ee;
	ee_pose_ = mech_.getEEPose(stepper_angle_current_); 
	
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "Articulated_Action");
	ArticulatedAction articulated("ik_action");

	return 0;		
}	