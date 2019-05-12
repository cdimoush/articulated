/*
ROS Stepper motor control for arduino nano.
*/

#include <simple_serial.h>
#include <Stepper.h>
#include <math.h>

SimpleSerial ser;
const int stepper_id = 2;
const int stepsPerRev = 200;
Stepper motor(stepsPerRev, 8, 9, 10, 11);


void setup() 
{
	Serial.begin(9600);
	ser.subscribe("stepper_id", "empty", sendStepperId);
	ser.subscribe("calibrate", "empty", calibrate);
	ser.subscribe("set_step_pos", "int", setStepperPos);

  pinMode(LED_BUILTIN, OUTPUT);
	motor.setSpeed(50);
}

void sendStepperId()
{
	ser.publish("stepper_id", String(stepper_id));
}

void calibrate()
{
	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on 
  delay(500);                       // wait for half a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off 
  delay(500);

  ser.publish("calibrate", "Calibration Complete!");
}

void setStepperPos(int steps)
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
    	ser.publish("step_feedback", String(i+1));
		delay(10);

	}
}

void loop() 
{
	ser.spin();
	delay(1);
}
