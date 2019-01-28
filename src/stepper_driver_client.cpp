#include "ros/ros.h"
#include "articulated/StepperDriver.h"

class StepperDriverClient
{
public:
  StepperDriverClient();

private:
  ros::NodeHandle nh_;

};


StepperDriverClient::StepperDriverClient()
{
  ros::ServiceClient client = nh_.serviceClient<articulated::StepperDriver>("stepper_driver_service");
  articulated::StepperDriver srv;
  
  for (int i = 0; i < 5; i++){

    srv.request.stepper_id = 0;
    srv.request.steps = 2;

    if (client.call(srv))
    {
      ROS_INFO("Service Success");
    }
    else
    {
      ROS_ERROR("Service Failed");
    }

    srv.request.stepper_id = 1;
    srv.request.steps = 5;

    if (client.call(srv))
    {
      ROS_INFO("Service Success");
    }
    else
    {
      ROS_ERROR("Service Failed");
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stepper_driver_client");

  StepperDriverClient step_client;
  return 0;
}