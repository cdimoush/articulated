#ifndef IKACTION_H
#define IKACTION_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <actionlib/server/simple_action_server.h>
#include <articulated/ikAction.h>

#include "articulated_action.h"


class ArticulatedAction::IkAction
{
	/*
private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<articulated::ikAction> as_;
	std::string action_name_;
	//create messages that are used to published feedback/result
	articulated::ikFeedback feedback_;
	articulated::ikResult result_;


public:
	void setServer(std::string name)
	{
		ROS_ERROR_STREAM(name);
		as_(name, false);
	}

	//~IkAction(void)
	//{	

	//}		
	
	//void executeCB(const articulated::ikGoalConstPtr &goal);
	*/
};
#endif