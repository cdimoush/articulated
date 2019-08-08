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
const int stepper_id = 0;
const int pulsePerRev = 6400;
const double qPerPulse = 2*PI / pulsePerRev;
const double feedback_resolution = PI/8;
const int time_delay = 100;

bool cal_pulse = false;


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

void calibrate(int dir)
{
  digitalWrite(ena_pin, LOW);
  if (dir < 0)
  {
    digitalWrite(dir_pin, LOW);
  }
  else
  {
    digitalWrite(dir_pin, HIGH);
  }
  cal_pulse = !cal_pulse;
  digitalWrite(pul_pin, cal_pulse);
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
    digitalWrite(dir_pin, LOW);
    dir_multi = -1;
  }
  else
  {
    digitalWrite(dir_pin, HIGH);
   dir_multi = 1;
  }

  //incrementally step to goal
  double pulses = pulsePerRev * fabs(dq)/(2*PI);
  Serial.print("Pulses: ");
  Serial.println(pulses);
  double angle_counter = 0;
  unsigned long t = micros();
  for(unsigned long int i = 0; i < pulses; i++)
  {
    while (time_delay > micros() - t)
    {
      //delayMicroseconds(1);
      continue;
      //Serial.println("Wait");
      //delayMicroseconds(5);
    }
    digitalWrite(pul_pin, HIGH);
    delayMicroseconds(time_delay);
    digitalWrite(pul_pin, LOW);
    t = micros();
    
    angle_counter = angle_counter + qPerPulse;

    /*
    if (angle_counter >= feedback_resolution)
    {
       ser.publish("step_feedback", String(dir_multi * angle_counter));
       angle_counter = 0;
       Serial.println(micros()-t);
    }
    */
    
  }
  if (angle_counter != 0)
  {
    ser.publish("step_feedback", String(dir_multi * angle_counter));
    ser.publish("step_goal", "1");
  }

}

void loop() 
{
  ser.spin();
  delay(1);
}
