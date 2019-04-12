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


class StepperDriverClient
{
public:
  StepperDriverClient();

private:
  void setStepperCallback(std_msgs::Float64MultiArray goal);
  void stepperService(double g_rad, int s_id);
  void serialCallback(articulated::serial_msg data);
  void ikPosCallback(geometry_msgs::Pose pos_goal);

  ros::NodeHandle nh_;
  ros::Subscriber stepper_angle_sub_;
  ros::Publisher ee_pub_;
  ros::Publisher joint_state_pub_;
  ros::ServiceClient client_;
  MechCalc mech_;

  //New Shit....
  ros::Subscriber serial_sub_;
  ros::Subscriber ik_pos_sub_;
  ros::Publisher serial_pub_;

  
  double stepper_angle_current_[2];
  double stepper_angle_goal_[2];
  geometry_msgs::Pose ee_pos_;
};


StepperDriverClient::StepperDriverClient()
{
  client_ = nh_.serviceClient<articulated::StepperDriver>("stepper_driver_service");
  stepper_angle_sub_ = nh_.subscribe("articulated/set_stepper_angle", 1000, &StepperDriverClient::setStepperCallback, this);
  ee_pub_ = nh_.advertise<geometry_msgs::Pose>("articulated/ee_pos", 1000);
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("articulated/joint_state", 1000);

  //New Shit.....
  serial_sub_ = nh_.subscribe("articulated/serial/receive", 1000, &StepperDriverClient::serialCallback, this);
  serial_pub_ = nh_.advertise<articulated::serial_msg>("articulated/serial/send", 1000);
  ik_pos_sub_ = nh_.subscribe("articulated/pos_goal", 1000, &StepperDriverClient::ikPosCallback, this);


  stepper_angle_current_[0] = M_PI/2;
  stepper_angle_current_[1] = 0;

  ee_pos_ = mech_.getEEPose(stepper_angle_current_); 
  ee_pub_.publish(ee_pos_);
  ROS_INFO("EEx: %f", ee_pos_.position.x);

  while (ros::ok())
  {
    joint_state_pub_.publish(mech_.joint_state_);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
}

void StepperDriverClient::serialCallback(articulated::serial_msg data)
{
  int i = data.micro_id;
  std::stringstream string_id; 
  string_id << i+1;

  int steps;
  std::stringstream s_steps (data.msg);
  s_steps >> steps;
  ROS_ERROR_STREAM("Steps: " << steps);
  
  double spr;
  nh_.getParam("articulated/stepper/" + string_id.str() + "/spr", spr);
  stepper_angle_current_[i] = stepper_angle_current_[i] + 2*M_PI * steps/(fabs(steps)*spr); 

  //DEBUG
  double angle_degree = stepper_angle_current_[i] * 180 / M_PI;
  //ROS_ERROR_STREAM("Stepper " << i << " angle: " << angle_degree);

  ee_pos_ = mech_.getEEPose(stepper_angle_current_); 
  ee_pub_.publish(ee_pos_);
}

void StepperDriverClient::ikPosCallback(geometry_msgs::Pose pose_goal)
{
  double * jt_st_goal;
  jt_st_goal = mech_.inverseKinematics(pose_goal, stepper_angle_current_);
  ROS_ERROR_STREAM("Return From IK");
  std_msgs::Float64MultiArray goal_msg_out;
  goal_msg_out.data.resize(2);
  goal_msg_out.data[0] = jt_st_goal[0];
  goal_msg_out.data[1] = jt_st_goal[1];
  ROS_ERROR_STREAM("Sending Goal to set stepper");
  setStepperCallback(goal_msg_out);
}
void StepperDriverClient::setStepperCallback(std_msgs::Float64MultiArray goal)
{
  for (int i = 0; i<2; i++)
  {
    if (goal.data[i] != stepper_angle_current_[i])
    {
      double g = goal.data[i] - stepper_angle_current_[i];
      //Publish Step Goal in this funtion for the time being
      
      //spr is steps per revolution, the spr for each stepper is a param
      double spr;
      std::stringstream string_id;
      std::stringstream g_string;
      string_id << i+1;
      nh_.getParam("articulated/stepper/" + string_id.str() + "/spr", spr);
      int g_steps = round(g*spr/(2*M_PI));
      g_string << g_steps;

      articulated::serial_msg g_msg;
      g_msg.micro_id = i;
      g_msg.topic = "set_step_pos";
      g_msg.msg = g_string.str();
      serial_pub_.publish(g_msg);
      ROS_ERROR_STREAM("sending stepper " << i + 1 << " goal of " << g_steps <<" steps");
    }
  }
}

void StepperDriverClient::stepperService(double g_rad, int s_id)
{
  //Function takes a stepper angle goal in radians (g_rad)
  //and the stepper id (s_id)

  //Create Service Call and give it the stepper id
  articulated::StepperDriver srv;
  srv.request.stepper_id = s_id;

  //Take the steps for rev number
  //determing the number of steps to reach goal
  //spr is steps per revolution, the spr for each stepper is a param
  double spr;
  std::stringstream string_id;
  string_id << s_id;
  nh_.getParam("articulated/stepper/" + string_id.str() + "/spr", spr);
  ROS_ERROR("Creating Service for Stepper  %i", s_id);
  ROS_ERROR("Goal of %f radians", g_rad);

  int g_steps = round(g_rad*spr/(2*M_PI));
  srv.request.steps = g_steps;

  if (client_.call(srv))
  {
    ROS_INFO("Service Success");
    stepper_angle_current_[s_id - 1]=stepper_angle_current_[s_id-1]+g_rad;
    ROS_ERROR("Stepper %i new state is %f", s_id, stepper_angle_current_[s_id-1]);
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