#include "mechanism_calculations.h"
//#include "geometry_msgs/Transform.h"

MechCalc::MechCalc()
{
	buildMech();
}
void MechCalc::buildMech()
{
	//Defined number of joints
	joint_state_.position.resize(2);
	//Get Mechanism Params
	nh_.getParam("/articulated/link/1/length", link1_);
	nh_.getParam("/articulated/link/2/length", link2_);
	nh_.getParam("/articulated/link/3/length", link3_);
	nh_.getParam("/articulated/stepper/2/x", stepper2_.coord[0]);
	nh_.getParam("/articulated/stepper/2/y", stepper2_.coord[1]);
	nh_.getParam("/articulated/stepper/2/hub", stepper2_.hub);

}
geometry_msgs::Pose MechCalc::forwardKinematics(double step_angle[2])
{
	//Do forward kinematics to get position of end effector
	//Mechanism design is laid out in this function
	//Consider pulling robot properties and mech from URDF

	stepper1_.angle = step_angle[0];
	stepper2_.angle = step_angle[1];

	//Joint Angles
	///////////////////
	//Angle 0 is angle of the stepper1 (0-90Deg)
	//Angle 1 has to be calculated from geometry of mechanism
	double q[2];
	q[0] = stepper1_.angle;
	//a - lenght of the link2 small lever
	//b - link3 lenght
	//c - distance between base of link3 and end of link1
	double a; nh_.getParam("/articulated/link/2/lever", a);
	double b = link3_;
	double x1 = link1_*cos(stepper1_.angle);
	double y1 = link1_*sin(stepper1_.angle);
	double x3 = stepper2_.coord[0] - stepper2_.hub*cos(stepper2_.angle);
	double y3 = stepper2_.coord[1] + stepper2_.hub*sin(stepper2_.angle);
	double c = sqrt(pow(x1-x3, 2) + pow(y1-y3, 2));
	//phi - angle btw c and a
	//lamda - angle btw b and c
	//gamma - angle btw c and horizontal 
	//theta - difference btw phi and gamma
	double phi = acos((c*c + a*a - b*b)/(2*c*a));
	double lamda = acos((c*c + b*b - a*a)/(2*c*b));
	double gamma = asin((y1 - y3)/c);
	double theta = fabs(phi - gamma);
	//theta is q1 in 0 frame, it must be in the 1 frame for transform math
	//Check if theta is above or below horizontal
	// Case 1: theta is below horizontal (or horizontal) bc end of link3 is above end of link1
	if ((y3 + link3_*sin(lamda+gamma)) >= y1) 
	{
		q[1] = -1*(q[0] + theta);
	}
	//Case 2: theta is above horizontal but less than q0
	else if (theta < q[0])
	{
		q[1] = -1*(q[0] - theta);
	}
	else
	{
		ROS_ERROR("CANNOT calculate joint states from mechanism state");
	}

	//Update Joints
	joint_state_.position[0] = q[0];
	joint_state_.position[1] = q[1];
	

	//Transform from 1 to 0
	Eigen::MatrixXf t01(4, 4);
	t01 << cos(q[0]), -sin(q[0]), 0, link1_*cos(q[0]),
		   sin(q[0]), cos(q[0]), 0, link1_*sin(q[0]),
		   0, 0, 1, 0,
		   0, 0, 0, 1; 
	//EE coord in frame 1
	Eigen::MatrixXf ee1(4, 1); ee1 <<  link2_*cos(q[1]), link2_*sin(q[1]), 0, 1;
	Eigen::VectorXf ee0(4, 1); ee0 = t01 * ee1;
	//t01 = t01 * ee1;
	
	geometry_msgs::Pose ee_pos;
	ee_pos.position.x = ee0(0, 0);
	ee_pos.position.y = ee0(1, 0);
	//ee_pos.position.z = ee0(0, 0);
	return ee_pos;
}
