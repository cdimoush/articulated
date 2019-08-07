#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <articulated/gripperAction.h>
#include "articulated/serial_msg.h"

class ArticulatedGripper
{
public:
	ArticulatedGripper(std::string gripper_server_name);
	~ArticulatedGripper();
private:
	//FUNCTIONS
	void gripCB(const articulated::gripperGoalConstPtr &goal);
	void serialCallback(articulated::serial_msg data);
	//OBJECTS
	ros::NodeHandle nh_;
	//PUBLISHERS / SUBSCRIBERS
	ros::Publisher serial_pub_;
	ros::Subscriber serial_sub_;
	//ACTION
	actionlib::SimpleActionServer<articulated::gripperAction> grip_as_;
	//VARS
	bool goal_state_;
	bool gripper_state_; //false -> open // true -> close
	double gripper_id_;
};

ArticulatedGripper::ArticulatedGripper(std::string gripper_server_name) :
	grip_as_(nh_, gripper_server_name, false)
{
	nh_.getParam("articulated/gripper/id", gripper_id_);
	nh_.getParam("articulated/gripper/state", gripper_state_);

	//Define PUBLISHERS
  	serial_pub_ = nh_.advertise<articulated::serial_msg>("articulated/serial/send", 1000);
	//Define SUBSCRIBERS
	serial_sub_ = nh_.subscribe("articulated/serial/receive", 1000, &ArticulatedGripper::serialCallback, this);
	
	grip_as_.start();

	while (ros::ok())
	{
		if (!grip_as_.isActive())
		{
			if (grip_as_.isNewGoalAvailable())
			{
				gripCB(grip_as_.acceptNewGoal());
			}
		}
		else
		{
			if (goal_state_ == gripper_state_)
			{
				nh_.setParam("articulated/gripper/state", gripper_state_);
				articulated::gripperResult result;
				result.close = gripper_state_;
				result.open = !result.close;
				grip_as_.setSucceeded(result);
			}
		}

		ros::spinOnce();	
		ros::Duration(0.01).sleep();
	}
}

ArticulatedGripper::~ArticulatedGripper(void)
{
}

void ArticulatedGripper::serialCallback(articulated::serial_msg data)
{
	if (data.micro_id == gripper_id_)
	{
		if (data.topic == "grip_feedback")
		{
			if (data.msg == "0")
			{
				ROS_INFO("Gripper Open");
				gripper_state_ = false;
			}
			else if (data.msg == "1")
			{
				ROS_INFO("Gripper Close");
				gripper_state_ = true;
			}
		}
	}
}

void ArticulatedGripper::gripCB(const articulated::gripperGoalConstPtr &goal)
{
	ROS_INFO("Gripper Goal Accepted");
	articulated::serial_msg g_msg;

	g_msg.micro_id = gripper_id_;

	//Determine gripper should be opened / closed / remain same
	//State (false -> open // true -> close)
	if (gripper_state_ && goal->open) //Gripper is closed & goal is to open
	{
		ROS_INFO("Opening Gripper");
		g_msg.topic = "open_gripper";
		goal_state_ = false;
		serial_pub_.publish(g_msg);
	}
	else if (!gripper_state_ && goal->close) //Gripper is open & goal is to close
	{
		ROS_INFO("Closing Gripper");
		g_msg.topic = "close_gripper";
		goal_state_ = true;
		serial_pub_.publish(g_msg);
	}
	else
	{
		ROS_ERROR_STREAM("GRIPPER GOAL == CURRENT GRIP STATE");
		ROS_ERROR_STREAM("CANCELING");
		grip_as_.setPreempted();
	}
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "Articulated_Gripper");
	ArticulatedGripper gripper("gripper_action");

	return 0;		
}	