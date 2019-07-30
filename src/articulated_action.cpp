#include "articulated_action.h"

ArticulatedAction::ArticulatedAction(std::string ik_server_name, std::string cal_server_name) :
	ik_as_(nh_, ik_server_name, false),
	ik_action_name_(ik_server_name),
	//ERROR LINKING CALLBACK FUNCTION
	cal_as_(nh_, cal_server_name, boost::bind(&ArticulatedAction::executeCalCB, this, _1),false)
	//cal_action_name_(cal_server_name) 

	//boost::bind(&ArticulatedAction::ikCB, this, _1)
{
	//Define PUBLISHERS
  	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("articulated/joint_state", 1000);
  	serial_pub_ = nh_.advertise<articulated::serial_msg>("articulated/serial/send", 1000);
	//Define SUBSCRIBERS
	serial_sub_ = nh_.subscribe("articulated/serial/receive", 1000, &ArticulatedAction::serialCallback, this);

	ik_as_.start();
	cal_as_.start();

	//SET INITIAL JOINT ANGLES
	//NOTE: This should be replaced with a calibration procedure
	stepper_angle_current_[0] = 0;
  	stepper_angle_current_[1] = M_PI/2;
  	stepper_angle_current_[2] = 0;
  	//Set Stepper States, 0->off 1->on
  	stepper_state_[0] = 0;
  	stepper_state_[1] = 0;
  	stepper_state_[2] = 0;
  	
  	ee_pose_ = mech_.getEEPose(stepper_angle_current_);
  	

  	
  	
	while (ros::ok())
	{
		if (ik_as_.isActive()) //Check if IK is currently processing goal
		{
			ikFB();
		}

		else if (ik_as_.isNewGoalAvailable()) //Check if new goal available
		{
			//ROS_ERROR_STREAM("NEW GOAL");
			ikCB(ik_as_.acceptNewGoal()); //Accept / Send goal to the IK Callback

		}

		//Publish Joint State
		joint_state_pub_.publish(mech_.joint_state_);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
}

ArticulatedAction::~ArticulatedAction()
{
	//FUNCTION IS NOT BEING CALLED
	//MAYBE TRY
	//signal(SIGINT,quitProcess);
	ROS_ERROR_STREAM("YOU KILLED THE ROBOT :/");
	ROS_ERROR_STREAM("");
	ROS_ERROR_STREAM("De-energizing steppers... ");
	for (int i = 0; i < 3; i++)
	{
		ROS_ERROR_STREAM("i");
		articulated::serial_msg s;
		s.micro_id = i;
		s.topic = "set_step_pos";
		s.msg = "toggle_hold";
		serial_pub_.publish(s);
	}

	ROS_ERROR_STREAM("Quiting ROS");
	ros::shutdown();
	exit(0);
}

void ArticulatedAction::ikCB(const articulated::ikGoalConstPtr &goal)
{
	double * jt_st_goal;
	bool flag;
	ik_goal_.position.x = goal->x;
	ik_goal_.position.y = goal->y;
	ik_goal_.position.z = goal->z;
	//CHECK IF GOAL IS LEGIT HERE. IF SO RUN INVERSE KINEMATICS
	std::tie(jt_st_goal, flag) = mech_.inverseKinematics(ik_goal_, stepper_angle_current_);
	if (!flag) //If Flag is high inverse kinematics failed
	{
		std_msgs::Float64MultiArray goal_msg_out;
		goal_msg_out.data.resize(3);
		goal_msg_out.data[0] = jt_st_goal[0];
		goal_msg_out.data[1] = jt_st_goal[1];
		goal_msg_out.data[2] = jt_st_goal[2];

		setSteppers(goal_msg_out);
	}
	else 
	{
		ROS_ERROR_STREAM("IK SOLUTION IS INACCURATE OR DOES NOT EXIST");
		ik_as_.setPreempted();
	}
}

void ArticulatedAction::ikFB()
{
	ik_feedback_.x = ee_pose_.position.x; 
	ik_feedback_.y = ee_pose_.position.y; 
	ik_feedback_.z = ee_pose_.position.z; 
	ik_as_.publishFeedback(ik_feedback_);

	for (int i=0; i<3; i++)
	{
		//Check if steppers are still working on goal
		if (stepper_state_[i] == 1)
			return;
	}

	//If steppers have completed goal
	ik_result_.x = ik_feedback_.x;
	ik_result_.y = ik_feedback_.y;
	ik_result_.z = ik_feedback_.z;
	ik_error_.position.x = ik_result_.x - ik_goal_.position.x;
	ik_error_.position.y = ik_result_.y - ik_goal_.position.y;
	ik_error_.position.z = ik_result_.z - ik_goal_.position.z;
	ik_as_.setSucceeded(ik_result_);
	ROS_ERROR_STREAM("IK SUCCESS");
	ROS_ERROR_STREAM("----------");
	ROS_ERROR_STREAM("position");
	ROS_ERROR_STREAM(ee_pose_.position);
	ROS_ERROR_STREAM("error");
	ROS_ERROR_STREAM(ik_error_.position);

}
void ArticulatedAction::setSteppers(std_msgs::Float64MultiArray goal)
{
	//Send Step goal (dq) to arduinos
	//ROS_ERROR_STREAM("Setting Stepper Angles");
	for (int i = 0; i<3; i++)
	{

		if (fabs(goal.data[i] - stepper_angle_current_[i]) > 0.005)
		{
			stepper_state_[i] = 1; // Turn Stepper on
			double g = goal.data[i] - stepper_angle_current_[i];
			ROS_ERROR_STREAM("Stepper " << i << " Rad Diff: " << g);


			std::stringstream g_string;
			g_string << g;
			//SET STEP POS arduino topic now takes goal as dq (in radians)
			articulated::serial_msg g_msg;
			g_msg.micro_id = i;
			g_msg.topic = "set_step_pos";
			g_msg.msg = g_string.str();

			//ROS_ERROR_STREAM("Stepper " << i << ": Sending Goal of " << g);
			serial_pub_.publish(g_msg);
		}
	}
}

void ArticulatedAction::serialCallback(articulated::serial_msg data)
{
	//FEEDBACK FROM SERIAL PORT, get stepper positions

	//Save incoming data... ID and Steps
	int i = data.micro_id;
	std::stringstream string_id; 
	string_id << i;

	if (data.topic == "step_feedback")
	{
		double dq;
		std::stringstream string_dq(data.msg);
		string_dq >> dq;
		stepper_angle_current_[i] = stepper_angle_current_[i] + dq;

		geometry_msgs::Pose ee;
		ee_pose_ = mech_.getEEPose(stepper_angle_current_); 
	}
	else if (data.topic == "step_goal")

		if(data.msg == "1")
		{
			//ROS_ERROR_STREAM("Stepper " << i << ": Goal Complete");
			stepper_state_[i] = 0;
		}
	
}

void ArticulatedAction::executeCalCB(const articulated::calibrateGoalConstPtr &goal)
{
	int kfd = 0;
	struct termios cooked, raw;

	//void (*keyQuitter)(int);
	//keyQuitter = signal(SIGINT,quitKeyLoop);

	char c;
	bool dirty=false;

	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("ENTERING CALIBRATION MODE");
	puts("---------------------------");
	puts("Use <-- and --> arrow keys to conrol joints");
	puts("Press Tab to switch to the next joint");
	puts("---------------------------");
	puts("");
	
	int i = 0;
	double dq = 0;
	double q = 0;

	puts("Calibrate 1st Stepper");
	while (i<3)
	{

		// get the next event from the keyboard  
		if(read(kfd, &c, 1) < 0)
		{
		  perror("read():");
		  exit(-1);
		}

		switch(c)
		{
		  case KEYCODE_L:
		  	dq = -1;
		    ROS_ERROR_STREAM("LEFT");
		    break;
		  case KEYCODE_R:
		  	dq = 1;
		    ROS_ERROR_STREAM("RIGHT");
		    break;
		  case KEYCODE_TAB:
		    dirty = true;
		    break;
		}
		
		if (dq != 0)
		{
			ROS_ERROR_STREAM("Publish to Arduino");
			q = M_PI/16 * dq;

			std::stringstream g_string;
			g_string << q;
			//SET STEP POS arduino topic now takes goal as dq (in radians)
			articulated::serial_msg g_msg;
			g_msg.micro_id = i;
			g_msg.topic = "set_step_pos";
			g_msg.msg = g_string.str();
			serial_pub_.publish(g_msg);

			dq = 0;
		}

		if(dirty ==true)
		{
			i ++;
			dirty=false;
			if (i<3)
				puts("Next Stepper");	
			else
				puts("Finishing Calibration");
		 }

	}


	tcsetattr(kfd, TCSANOW, &cooked);
	
	stepper_angle_current_[0] = 0;
  	stepper_angle_current_[1] = M_PI/2;
  	stepper_angle_current_[2] = 0;
  	ee_pose_ = mech_.getEEPose(stepper_angle_current_);	

  	articulated::calibrateResult result;
	cal_as_.setSucceeded(result);
}

/*
void quitProcess(int sig)
{
	//If Articulated Action is killed with key interupt
	(void)sig;

	for (int i = 0; i < 3; i++)
	{
		articulated::serial_msg s;
		s.micro_id = i;
		s.topic = "set_step_pos";
		s.msg = "toggle_hold";
		serial_pub_.publish(s);
	}

	ros::shutdown();
	exit(0);
}
*/

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "Articulated_Action");
	ArticulatedAction articulated("ik_action", "calibration_action");

	return 0;		
}	