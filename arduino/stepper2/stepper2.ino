/*
ROS Stepper motor control for arduino nano.
*/

#include <simple_serial.h>
#include <math.h>

//Arduino Pins
const int ena_pin = 2;
const int dir_pin = 3;
const int pul_pin = 4;

//Vars
SimpleSerial ser;
const int stepper_id = 2;
const int pulsePerRev = 1600;
const double qPerPulse = 2*PI / pulsePerRev;
const double feedback_resolution = PI/16;
const int time_delay = 1000;


void setup() 
{
  Serial.begin(9600);
  ser.subscribe("stepper_id", "empty", sendStepperId);
  ser.subscribe("calibrate", "empty", calibrate);
  ser.subscribe("toggle_hold", "empty", toggleHold);
  ser.subscribe("set_step_pos", "double", setStepperPos);
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ena_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(pul_pin, OUTPUT);
  digitalWrite(ena_pin, LOW); //LOW HOLDS STEPPER POSITION
  digitalWrite(dir_pin, LOW);
  digitalWrite(pul_pin, HIGH);
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

void toggleHold()
{
  digitalWrite(ena_pin, !digitalRead(ena_pin));
}

void setStepperPos(double dq)
{
  //Enable Motor
  digitalWrite(ena_pin, LOW);
  //Determine cw or ccw rotation
  int dir_multi;
  if (dq < 0)
  {
    digitalWrite(dir_pin, HIGH);
    dir_multi = -1;
  }
  else
  {
    digitalWrite(dir_pin, LOW);
   dir_multi = 1;
  }

  //incrementally step to goal
  double pulses = pulsePerRev * fabs(dq)/(2*PI);
  double angle_counter = 0;
  for(int i = 0; i < pulses; i++)
  {
    digitalWrite(pul_pin, HIGH);
    delayMicroseconds(time_delay);
    digitalWrite(pul_pin, LOW);
    delayMicroseconds(time_delay);
    angle_counter = angle_counter + qPerPulse;
    if (angle_counter >= feedback_resolution)
    {
       ser.publish("step_feedback", String(dir_multi * angle_counter));
       angle_counter = 0;
    }
  }
  if (angle_counter != 0)
  {
    ser.publish("step_feedback", String(dir_multi * angle_counter));
  }

}

void loop() 
{
  ser.spin();
  delay(1);
}
