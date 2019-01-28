/*
ROS Stepper motor control for arduino nano. Talks with PC via serial com.
When 0 is recieved motor calibrates and returns it's unique ID.
*/


#include <Stepper.h>
#include <math.h>

const int stepper_id = 0;
const int stepsPerRev = 200;
Stepper motor(stepsPerRev, 2, 3, 4, 5);


void setup() 
{
	Serial.begin(9600);
	motor.setSpeed(60);
}

void loop() 
{
	if (Serial.available())
	{
		String raw_incoming_msg = Serial.readString();
		float incoming_msg = raw_incoming_msg.toFloat();
    //delay(5);
		if (incoming_msg == 0) //calibrate
		{
			calibrate();
		}

		else
		{
			control_servo(incoming_msg);
		}
	}
}


void calibrate()
{
	Serial.println(stepper_id);
	//should now move to limit switch and go through a calibration
}

void control_servo(int steps)
{
	//Determine cw or ccw rotation
	int dir = 0;
	if (steps < 0)
	{
		dir = -1;
	}
	else
	{
		dir = 1;
	}

	//incrementally step to goal
	for(int i = 0; i < fabs(steps); i++)
	{
		motor.step(1 * dir);
    Serial.println(1); //Tell computer that goal has been completed 
		delay(5);

	}
  Serial.println(0);
}
