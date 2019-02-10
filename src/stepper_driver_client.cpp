#include "ros/ros.h"
#include "articulated/StepperDriver.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <sstream>
#include <string>


class StepperDriverClient
{
public:
  StepperDriverClient();

private:
  void jointStateCallback(std_msgs::Float64MultiArray jt_goal);
  void stepperService(float goal_degrees, int s_id);

  ros::NodeHandle nh_;
  ros::Subscriber joint_sub_;
  ros::ServiceClient client_;
  float jt_state_[2];

};


StepperDriverClient::StepperDriverClient()
{
  jt_state_[0] = 90;
  jt_state_[1] = 90;
  client_ = nh_.serviceClient<articulated::StepperDriver>("stepper_driver_service");
  joint_sub_ = nh_.subscribe("articulated/joint_state_goal", 1000, &StepperDriverClient::jointStateCallback, this);
  
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void StepperDriverClient::jointStateCallback(std_msgs::Float64MultiArray jt_goal)
{
  ROS_ERROR("Jt Goal Recieved");
  for (int i = 0; i<2; i++)
  {
    if (jt_goal.data[i] != jt_state_[i])
    {
      float g = jt_goal.data[i] - jt_state_[i];
      stepperService(g, i+1);
    }
  }
}

void StepperDriverClient::stepperService(float goal_degrees, int s_id)
{
  articulated::StepperDriver srv;

  srv.request.stepper_id = s_id;
  //Take the steps for rev number
  //determing the number of steps for change int degrees
  double spr;
  std::stringstream string_id;
  string_id << s_id;
  nh_.getParam("articulated/stepper/" + string_id.str() + "/spr", spr);
  ROS_ERROR("Creating Service for Stepper  %i", s_id);
  ROS_ERROR("Goal of %f degrees", goal_degrees);

  int goal_steps = round(goal_degrees*spr/360);

  srv.request.steps = goal_steps;

  if (client_.call(srv))
  {
    ROS_INFO("Service Success");
    jt_state_[s_id - 1]=jt_state_[s_id-1]+goal_degrees;
    ROS_ERROR("Stepper %i new state is %f", s_id, jt_state_[s_id-1]);
  }
  else
  {
    ROS_ERROR("Service Failed");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stepper_driver_client");
  StepperDriverClient step_client;
  return 0;
}