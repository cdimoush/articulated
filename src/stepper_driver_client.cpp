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
  void serialCallback(articulated::serial_msg data);
  void ikPosCallback(geometry_msgs::Pose pos_goal);
  int calcSteps(double angle, double stepper_id);

  ros::NodeHandle nh_;
  ros::Subscriber stepper_angle_sub_;
  ros::Publisher ee_pub_;
  ros::Publisher joint_state_pub_;
  ros::ServiceClient client_;
  MechCalc mech_;

  ros::Subscriber serial_sub_;
  ros::Subscriber ik_pos_sub_;
  ros::Publisher serial_pub_;
  
  double stepper_angle_current_[3];
  double stepper_angle_goal_[3];
  geometry_msgs::Pose ee_pos_;
};


StepperDriverClient::StepperDriverClient()
{
  stepper_angle_sub_ = nh_.subscribe("articulated/set_stepper_angle", 1000, &StepperDriverClient::setStepperCallback, this);
  ee_pub_ = nh_.advertise<geometry_msgs::Pose>("articulated/ee_pos", 1000);
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("articulated/joint_state", 1000);
  serial_sub_ = nh_.subscribe("articulated/serial/receive", 1000, &StepperDriverClient::serialCallback, this);
  serial_pub_ = nh_.advertise<articulated::serial_msg>("articulated/serial/send", 1000);
  ik_pos_sub_ = nh_.subscribe("articulated/pos_goal", 1000, &StepperDriverClient::ikPosCallback, this);

  stepper_angle_current_[0] = 0;
  stepper_angle_current_[1] = M_PI/2;
  stepper_angle_current_[2] = 0;

  ee_pos_ = mech_.getEEPose(stepper_angle_current_); 
  ee_pub_.publish(ee_pos_);

  while (ros::ok())
  { 
    joint_state_pub_.publish(mech_.joint_state_);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_ERROR_STREAM("Killing MAin");
}

void StepperDriverClient::serialCallback(articulated::serial_msg data)
{
  //FEEDBACK FROM SERIAL PORT, get stepper positions

  //Save incoming data... ID and Steps
  int i = data.micro_id;
  std::stringstream string_id; 
  string_id << i;

  /*int steps;
  std::stringstream s_steps(data.msg);
  s_steps >> steps;
  
  //Calcs new stepper angle
  double spr;
  nh_.getParam("articulated/stepper/" + string_id.str() + "/spr", spr);
  stepper_angle_current_[i] = stepper_angle_current_[i] + 2*M_PI * steps/spr; */

  double dq;
  std::stringstream string_dq(data.msg);
  string_dq >> dq;
  stepper_angle_current_[i] = stepper_angle_current_[i] + dq;

  //Debug
  //ROS_ERROR_STREAM("Stepper " << i << " angle: " << stepper_angle_current_[i]);

  ee_pos_ = mech_.getEEPose(stepper_angle_current_); 
  ee_pub_.publish(ee_pos_);
}

void StepperDriverClient::ikPosCallback(geometry_msgs::Pose pose_goal)
{
  double * jt_st_goal;
  bool flag;

  std::tie(jt_st_goal, flag) = mech_.inverseKinematics(pose_goal, stepper_angle_current_);
  if (!flag)
  {
    std_msgs::Float64MultiArray goal_msg_out;
    goal_msg_out.data.resize(3);
    goal_msg_out.data[0] = jt_st_goal[0];
    goal_msg_out.data[1] = jt_st_goal[1];
    goal_msg_out.data[2] = jt_st_goal[2];

    //DEBUG
    /*ROS_ERROR_STREAM("GOALS");
    ROS_ERROR_STREAM("JT0: " << jt_st_goal[0]);
    ROS_ERROR_STREAM("JT1: " << jt_st_goal[1]);
    ROS_ERROR_STREAM("JT2: " << jt_st_goal[2]);*/

    setStepperCallback(goal_msg_out);
  }
  else ROS_ERROR_STREAM("IK SOLUTION IS INACCURATE OR DOES NOT EXIST");
}
void StepperDriverClient::setStepperCallback(std_msgs::Float64MultiArray goal)
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

int StepperDriverClient::calcSteps(double angle, double stepper_id)
{
  double spr;
  if (stepper_id == 0)
  {
    spr = mech_.stepper0_.spr;
    return angle*spr/(2*M_PI);
  }
  if (stepper_id == 1)
  {
    spr = mech_.stepper1_.spr;
    return angle*spr/(2*M_PI);
  }
  if (stepper_id == 2)
  {
    spr = mech_.stepper2_.spr;
    return angle*spr/(2*M_PI);
  }
  else
  {
    ROS_ERROR_STREAM("COULD NOT GET SPR");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stepper_driver_client");
  StepperDriverClient step_client;
  return 0;
}