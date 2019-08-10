#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/JointState.h"
#include <string>
#include <vector>
#include <sstream>
#include <math.h>

class ArticulatedTfBroadcaster
{
  public:
    ArticulatedTfBroadcaster();

  private:
    void jtCallback(sensor_msgs::JointState j_state);
    void broadcastTf();
    void build();

    ros::NodeHandle nh_;
    tf::TransformBroadcaster broadcaster_;
    ros::Subscriber joint_sub_;

    tf::tfMessage joint_transform_;
    int t_; //number of transforms
};

ArticulatedTfBroadcaster::ArticulatedTfBroadcaster()
{
  joint_sub_ = nh_.subscribe("articulated/joint_state", 10, &ArticulatedTfBroadcaster::jtCallback, this);
  
  ros::Duration(1).sleep();build();
  while (ros::ok())
  {
    broadcastTf();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void ArticulatedTfBroadcaster::build()
{
  //SET TF FRAME IDS AND INIT TRANSFROMS
  /////////////////////////////////////////////

  //number of joints... Get number from parameters (Add 1 if using gripper)
  //number of transforms.... world->ee... so j+2
  int j; 
  bool gripper_bool;
  nh_.getParam("/articulated/joints", j);
  nh_.getParam("/articulated/gripper/bool", gripper_bool);
  
  t_ = j + 2;  
  if (gripper_bool)
  {
    ROS_ERROR_STREAM("TF_BR: CREATING GRIPPER TF");
    t_ ++;
  }

  joint_transform_.transforms.resize(t_);

  //id_list stores all the frame_ids for the tfs
  std::vector<std::string> id_list;
  id_list.push_back("world");
  id_list.push_back("base");
  //get joint names from parameter server
  for (int i = 0; i < j; i++)
  {
    std::stringstream j_num; 
    std::string j_id;
    j_num << i;
    nh_.getParam("articulated/joint/" + j_num.str() + "/id", j_id);
    id_list.push_back(j_id);
  }
  id_list.push_back("ee");
  if (gripper_bool)
  {
    id_list.push_back("gripper");
  }
  //init all translation and rotation tfs as blanks
  for(int i = 0; i < t_; i ++)
  {
    joint_transform_.transforms[i].header.frame_id = id_list[i];
    joint_transform_.transforms[i].child_frame_id = id_list[i + 1];
    joint_transform_.transforms[i].transform.translation.x = 0;
    joint_transform_.transforms[i].transform.translation.y = 0;
    joint_transform_.transforms[i].transform.translation.z = 0;
    joint_transform_.transforms[i].transform.rotation.x = 0;
    joint_transform_.transforms[i].transform.rotation.y = 0;
    joint_transform_.transforms[i].transform.rotation.z = 0;
    joint_transform_.transforms[i].transform.rotation.w = 1;
  }
  /////////////////////////////////////////////


  //GET AND SET STATIC TRANSFORMS
  /////////////////////////////////////////////

  //get static transformations
  std::vector<double>static_tran_list;
  double s_tran;
  nh_.getParam("articulated/base/transform", s_tran);
  static_tran_list.push_back(s_tran);
  //ERROR IN DAE CORDINATE DEFINITION
  static_tran_list.push_back(0); //no static trans from base -> slide
  for (int i = 0; i < j; i++)
  {
    std::stringstream j_num; 
    j_num << i;
    nh_.getParam("articulated/joint/" + j_num.str() + "/transform", s_tran);
    //ROS_ERROR_STREAM(s_tran); 
    static_tran_list.push_back(s_tran);
  }
  if (gripper_bool)
  {
    float gripper_transform;
    nh_.getParam("articulated/gripper/transform", gripper_transform);
    static_tran_list.push_back(gripper_transform);
  }
  //set statics transforms 
  for(int i = 0; i < t_; i ++)
  {
    //world - > base, slide -> joint 1
    if (i < 3)
    {
      joint_transform_.transforms[i].transform.translation.y = static_tran_list[i];
    }
    else
    {
      joint_transform_.transforms[i].transform.translation.x = static_tran_list[i];
    }
  }
  /////////////////////////////////////////////
}

void ArticulatedTfBroadcaster::jtCallback(sensor_msgs::JointState j_state)
{
  //save joint state
  std::vector<float> q;
  q.push_back(j_state.position[0]);
  q.push_back(j_state.position[1]);
  q.push_back(j_state.position[2]);
  
  //Update Z Translation
  double stepper0_spr;
  double stepper0_hub;
  nh_.getParam("/articulated/stepper/0/hub", stepper0_hub);
  joint_transform_.transforms[1].transform.translation.z = q[0]*stepper0_hub;

  //Joint 1 Rotation
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(q[1]);
  joint_transform_.transforms[2].transform.rotation.x = quat.x;
  joint_transform_.transforms[2].transform.rotation.y = quat.y;
  joint_transform_.transforms[2].transform.rotation.z = quat.z;
  joint_transform_.transforms[2].transform.rotation.w = quat.w;

  //Joint 2 Rotation
  quat = tf::createQuaternionMsgFromYaw(q[2]);
  joint_transform_.transforms[3].transform.rotation.x = quat.x;
  joint_transform_.transforms[3].transform.rotation.y = quat.y;
  joint_transform_.transforms[3].transform.rotation.z = quat.z;
  joint_transform_.transforms[3].transform.rotation.w = quat.w;
}

void ArticulatedTfBroadcaster::broadcastTf()
{
  //Publish All Transforms in list
  for(int i = 0; i < t_; i++)
  {
    tf::StampedTransform t_out;
    transformStampedMsgToTF(joint_transform_.transforms[i], t_out);
    broadcaster_.sendTransform(t_out);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "articulated_tf_broadcaster");
  ArticulatedTfBroadcaster b;
  return 0;
}