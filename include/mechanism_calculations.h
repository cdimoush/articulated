#include "ros/ros.h"
#include "articulated/StepperDriver.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Eigenvalues>




class MechCalc
{
public:
	MechCalc();
	geometry_msgs::Pose getEEPose(double step_angle[3]);
	double * inverseKinematics(geometry_msgs::Pose ee_pos_goal, double step_angle[3]);

	struct Stepper
	{
		//Hub radius and offset of coordinates are need for mech calc
		double hub; 
		double spr; 
		double angle;
		double torque;
		double coord[2];
	};

	sensor_msgs::JointState joint_state_;
	MechCalc::Stepper stepper0_, stepper1_, stepper2_;
	
private:
	//functions
	void buildMech();
	Eigen::MatrixXd buildJacobian(double q[2]);
	geometry_msgs::Pose forwardKinematics(sensor_msgs::JointState jt);
	double * calcStepperAngles(sensor_msgs::JointState jt, int max_it, double accuracy);
	double calcTheta(double angle[2]);
	
	//Objects
	ros::NodeHandle nh_;

	//Variables 
	#define PI 3.14159265359;

	double link1_;
	double link2_;
	double link3_;
	double stepper0_angle_;
	double stepper1_angle_;
	double stepper2_angle__;

	
	
};
