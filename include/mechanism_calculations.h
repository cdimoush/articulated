#include "ros/ros.h"
#include "articulated/StepperDriver.h"

#include <math.h>
#include <eigen3/Eigen/Dense>



class MechCalc
{
public:
	MechCalc();

	struct Stepper
	{
		//Orignally created for stepper 2
		//Hub radius and offset of coordinates are need for mech calc
		double hub; 
		double angle;
		double torque;
		double coord[2];
	};


private:
	//functions
	void forwardKinematics();
	void buildMech();

	//Objects
	ros::NodeHandle nh_;

	//Variables 
	#define PI 3.14159265359;

	double link1_;
	double link2_;
	double link3_;
	double stepper1_angle_;
	double stepper2_angle__;

	MechCalc::Stepper stepper1_, stepper2_;
	
};
