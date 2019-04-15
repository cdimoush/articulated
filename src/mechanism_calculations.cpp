#include "mechanism_calculations.h"
//#include "geometry_msgs/Transform.h"

MechCalc::MechCalc()
{
	buildMech();
}
void MechCalc::buildMech()
{
	//Defined number of joints
	joint_state_.position.resize(3);
	//Get Mechanism Params
	nh_.getParam("/articulated/link/1/length", link1_);
	nh_.getParam("/articulated/link/2/length", link2_);
	nh_.getParam("/articulated/link/3/length", link3_);
	nh_.getParam("/articulated/stepper/0/hub", stepper0_.hub);
	nh_.getParam("/articulated/stepper/0/spr", stepper0_.spr);
	nh_.getParam("/articulated/stepper/1/spr", stepper1_.spr);
	nh_.getParam("/articulated/stepper/2/x", stepper2_.coord[0]);
	nh_.getParam("/articulated/stepper/2/y", stepper2_.coord[1]);
	nh_.getParam("/articulated/stepper/2/hub", stepper2_.hub);
	nh_.getParam("/articulated/stepper/2/spr", stepper2_.spr);
	

}
geometry_msgs::Pose MechCalc::getEEPose(double step_angle[3])
{
	//Do forward kinematics to get position of end effector
	//Mechanism design is laid out in this function
	//Consider pulling robot properties and mech from URDF

	stepper0_.angle = step_angle[0];
	stepper1_.angle = step_angle[1];
	stepper2_.angle = step_angle[2];

	double q[3];
	q[0] = stepper0_.angle;
	q[1] = stepper1_.angle;
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
	//double theta = fabs(phi - gamma);
	double theta = fabs(phi - gamma);
	//theta is q1 in 0 frame, it must be in the 1 frame for transform math
	//Check if theta is above or below horizontal
	// Case 1: theta is below horizontal (or horizontal) bc end of link3 is above end of link1

	if ((y3 + link3_*sin(lamda+gamma)) >= y1) 
	{
		q[2] = -1*(q[1] + theta);
	}
	//Case 2: theta is above horizontal but less than q0
	else if (theta < q[0])
	{
		q[2] = -1*(q[1] - theta);
	}
	else
	{
		ROS_ERROR("CANNOT calculate joint states from mechanism state");
	}

	//Update Joints
	joint_state_.position[0] = q[0];
	joint_state_.position[1] = q[1];
	joint_state_.position[2] = q[2];
	
	return forwardKinematics(joint_state_);
}

geometry_msgs::Pose MechCalc::forwardKinematics(sensor_msgs::JointState jt)
{
	double q[3];
	q[0] = jt.position[0];
	q[1] = jt.position[1];
	q[2] = jt.position[2];

	//Transform from 1 to 0
	Eigen::MatrixXf t01(4, 4);
	t01 << cos(q[1]), -sin(q[1]), 0, link1_*cos(q[1]),
		   sin(q[1]), cos(q[1]), 0, link1_*sin(q[1]),
		   0, 0, 1, 0,
		   0, 0, 0, 1; 
	//EE coord in frame 1
	Eigen::MatrixXf ee1(4, 1); ee1 <<  link2_*cos(q[1]), link2_*sin(q[1]), 0, 1;
	Eigen::VectorXf ee0(4, 1); ee0 = t01 * ee1;
	
	geometry_msgs::Pose ee_pos;
	ee_pos.position.x = ee0(0, 0);
	ee_pos.position.y = ee0(1, 0);
	ee_pos.position.z = q[0]*stepper0_.hub; //REVs * 2*pi*r ---> (q/(2*pi))*(2*pi*r)
	//ROS_ERROR_STREAM("Returned EE Pose Z: " << ee_pos.position.z);

	return ee_pos;
}

double * MechCalc::inverseKinematics(geometry_msgs::Pose ee_pos_goal, double step_angle[3])
{
	ROS_ERROR_STREAM("Start IK");
	double q[3];
	double q_planar[2];
	geometry_msgs::Pose ee_pos_current = forwardKinematics(joint_state_);
	q[0] = joint_state_.position[0];
	q[1] = joint_state_.position[1];
	q[2] = joint_state_.position[2];
	q_planar[0] = q[1];
	q_planar[1] = q[2];
	sensor_msgs::JointState joint_state_goal;
	joint_state_goal.position.resize(3);

	//Solve Z
	double delta_z = ee_pos_goal.position.z - ee_pos_current.position.z;
	joint_state_goal.position[0] = q[0] + delta_z/stepper0_.hub;

	//Solve Planar
	Eigen::VectorXd delta_theta(2);
	Eigen::VectorXd delta_x(2);
	Eigen::VectorXd delta_pos(2);
	delta_pos[0] = ee_pos_goal.position.x - ee_pos_current.position.x;
	delta_pos[1] = ee_pos_goal.position.y - ee_pos_current.position.y;
	ROS_ERROR_STREAM("Delta x: "<< delta_pos[0]);
	ROS_ERROR_STREAM("Delta y: "<< delta_pos[1]);
	//Calc # of steps based on largest delta pos value
	double step_size = 0.0005;
	double delta_max = 0; 
	int steps;
	for (int i = 0; i < 2; i++)
	{
		if (fabs(delta_pos[i]) < step_size)
		{
			delta_pos[i] = 0;
		}
		if (fabs(delta_pos[i]) > delta_max)
		{
			delta_max = fabs(delta_pos[i]);
		}

	}
	steps = round(delta_max/step_size);
	ROS_ERROR_STREAM("IK Steps: "<< steps);

	delta_x = delta_pos / steps;
	for (int i = 0; i < steps; i ++)
	{
		Eigen::MatrixXd j = buildJacobian(q_planar);
		Eigen::MatrixXd j_inv = j.transpose() * (j*j.transpose()).inverse();
		//ROS_ERROR_STREAM(j);
		//ROS_ERROR_STREAM(j_inv);
		delta_theta = j_inv * delta_x;
		q[1] += delta_theta[0];
		q[2] += delta_theta[1];
	}

	
	joint_state_goal.position[1] = q[1];
	joint_state_goal.position[2] = q[2];
	double * step_angle_goal;
	step_angle_goal = calcStepperAngles(joint_state_goal, 1000, 0.0001);

	//geometry_msgs::Pose ee_ik = getEEPose(step_angle_goal);
	//ROS_ERROR_STREAM("Returned EE Pose X: " << ee_ik.position.x << ", " << ee_ik.position.y);

	return step_angle_goal;
}

Eigen::MatrixXd MechCalc::buildJacobian(double q[2])
{
	//linear velocity component of jacobian
	Eigen::MatrixXd jv(2, 2);
	jv << -link1_*sin(q[0]) - link2_*sin(q[0]+q[1]), -link1_*sin(q[0]+q[1]),
		  link1_*cos(q[0]) + link2_*cos(q[0]+q[1]), link1_*cos(q[0]+q[1]);

	return jv;
}

double * MechCalc::calcStepperAngles(sensor_msgs::JointState jt, int max_it, double accuracy)
{
	double q[3];
	static double step_angle[3];
	q[0] = jt.position[0];
	q[1] = jt.position[1];
	q[2] = jt.position[2];
	step_angle[0] = q[0];
	step_angle[1] = q[1];

	//CALC THETA FOR KNOWN JOINT ANGLE COMBINATION
	double theta;
	//Below Horizontal
	if (q[1] + q[2] <= 0)
	{
		theta = -1*(q[1] + q[2]);
	}
	//Above Horizontal but Less than q[1] (dont know if test is needed)
	else
	{
		theta = q[1] + q[2];
	}

	double t[2];
	double step_angle_guess[2][2] = {{q[1], -1.6}, {q[1], 1.6}};
	double angle1;
	//SECANT METHOD.... find step angle to that results in known theta
	//Looking for convergance with 0
	t[0] = calcTheta(step_angle_guess[0]) - theta;
	t[1] = calcTheta(step_angle_guess[1]) - theta;
	for (int i = 0; i < max_it; i++)
	{
		angle1 = (step_angle_guess[1][1]*t[0] - step_angle_guess[0][1]*t[1])/(t[0]-t[1]);

		if (t[1] < accuracy)
		{
			break;
		}
		else
		{
			step_angle_guess[1][1] = step_angle_guess[0][1];
			step_angle_guess[0][1] = angle1;
			t[1] = t[0];
			t[0] = calcTheta(step_angle_guess[0]) - theta;
		}
	}
	step_angle[2] = angle1;
	return step_angle;
}

double MechCalc::calcTheta(double angle[2])
{
	//Joint Angles
	///////////////////
	//Angle 0 is angle of the stepper1 (0-90Deg)
	//Angle 1 has to be calculated from geometry of mechanism
	//a - lenght of the link2 small lever
	//b - link3 lenght
	//c - distance between base of link3 and end of link1
	double a; nh_.getParam("/articulated/link/2/lever", a);
	double b = link3_;
	double x1 = link1_*cos(angle[0]);
	double y1 = link1_*sin(angle[0]);
	double x3 = stepper2_.coord[0] - stepper2_.hub*cos(angle[1]);
	double y3 = stepper2_.coord[1] + stepper2_.hub*sin(angle[1]);
	double c = sqrt(pow(x1-x3, 2) + pow(y1-y3, 2));
	//phi - angle btw c and a
	//lamda - angle btw b and c
	//gamma - angle btw c and horizontal 
	//theta - difference btw phi and gamma
	double phi = acos((c*c + a*a - b*b)/(2*c*a));
	double lamda = acos((c*c + b*b - a*a)/(2*c*b));
	double gamma = asin((y1 - y3)/c);
	//double theta = fabs(phi - gamma);
	double theta = fabs(phi - gamma);
	return theta;
}