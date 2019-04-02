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
  build();
  while (ros::ok())
  {
    broadcastTf();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}

void ArticulatedTfBroadcaster::build()
{
  int j; //number of joints
  nh_.getParam("/articulated/joints", j);
  t_ = j + 2; //number of transforms.... world->ee
  joint_transform_.transforms.resize(t_);

  std::vector<std::string> id_list;
  id_list.push_back("world");
  id_list.push_back("base");
  //add links (consider using parameters)
  id_list.push_back("link1");
  id_list.push_back("link2");
  id_list.push_back("ee");
  /*
  for(int i = 0; i < j; i++)
  {
    std::stringstream j_num;
    j_num << i + 1;
    std::string j_id;
    nh_.getParam("/articulated/joint/" + j_num.str() + "/id", j_id);
    id_list.push_back(j_id);
  }*/
  

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
}

void ArticulatedTfBroadcaster::jtCallback(sensor_msgs::JointState j_state)
{
  //Here is what you gotta do.....

  //Add first Rotation term to the i+1 tf, then rotation and translation to i+2, then just translation to ee
  std::vector<float> q;
  q.push_back(j_state.position[0]);
  q.push_back(j_state.position[1]);
  q.push_back(0);
  //Update Translation and Rotation
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(q[0]);
  joint_transform_.transforms[1].transform.rotation.x = quat.x;
  joint_transform_.transforms[1].transform.rotation.y = quat.y;
  joint_transform_.transforms[1].transform.rotation.z = quat.z;
  joint_transform_.transforms[1].transform.rotation.w = quat.w;
  for(int i = 0; i < t_ - 2; i++)
  {
    //Note: Joint transforms start at index 2 (0:world, 1:base)
    //float q = j_state.position[i];
    std::stringstream j_num;
    j_num << i + 1;
    double trans_mag;
    nh_.getParam("/articulated/joint/" + j_num.str() + "/transform", trans_mag);
    joint_transform_.transforms[i+2].transform.translation.x = trans_mag;
    //joint_transform_.transforms[i+2].transform.translation.x = trans_mag * cos(q[i]);
    //joint_transform_.transforms[i+2].transform.translation.y = trans_mag * sin(q[i]);
    
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(q[i+1]);
    joint_transform_.transforms[i+2].transform.rotation.x = quat.x;
    joint_transform_.transforms[i+2].transform.rotation.y = quat.y;
    joint_transform_.transforms[i+2].transform.rotation.z = quat.z;
    joint_transform_.transforms[i+2].transform.rotation.w = quat.w;
  }
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