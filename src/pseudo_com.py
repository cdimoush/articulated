#! /usr/bin/env python

import os
import rospy
import time
import serial

from articulated.msg import serial_msg

class SerialCom:
	_stepper_states = [0, 0, 0]
	_stepper_goals = [0, 0, 0]
	dq = 0.05
	def __init__(self):
		#Name of serial device folder and potential names given to arduinos
		rospy.loginfo('%s: Initializing', rospy.get_name())

		self._pub = rospy.Publisher('articulated/serial/receive', serial_msg, queue_size=10)
		rospy.Subscriber('articulated/serial/send', serial_msg, self.sendMsgCallback)
		self.main()

	def main(self):
		while not rospy.is_shutdown():
			if self._stepper_states != self._stepper_goals:
				for i in range(3):
					x = self._stepper_goals[i] - self._stepper_states[i]
					if x != 0: 
						#Shorter Delay for stepper 1
						if i == 1:
							rospy.Rate(450).sleep()
						else:
							rospy.Rate(25).sleep()
						if abs(x) >= self.dq:
							receive_msg = serial_msg()
							receive_msg.micro_id = i
							receive_msg.topic = "step_feedback"
							if x > 0:
								self._stepper_states[i] += self.dq
								receive_msg.msg = str(self.dq)
							if x < 0:
								self._stepper_states[i] -= self.dq
								receive_msg.msg = str(-1*self.dq)

							self._pub.publish(receive_msg)	
						else:
							goal_msg = serial_msg()
							goal_msg.micro_id = i
							goal_msg.topic = "step_goal"
							goal_msg.msg = str(1)
							self._pub.publish(goal_msg)

						
			else:
				for i in range(3):
					if self._stepper_states[i] != 0:
						self._stepper_states[i] = 0
						self._stepper_goals[i] = 0
				rospy.Rate(500).sleep()

	def sendMsgCallback(self, send_msg):
		#Intercept arduino_msg
		if send_msg.topic == "set_step_pos":
			self._stepper_goals[send_msg.micro_id] = float(send_msg.msg)
		if send_msg.topic == "open_gripper":
			rospy.Rate(500).sleep()
			goal_msg = serial_msg()
			goal_msg.micro_id = send_msg.micro_id
			goal_msg.topic = "grip_feedback"
			goal_msg.msg = str(0)
			self._pub.publish(goal_msg)
		if send_msg.topic == "close_gripper":
			rospy.Rate(500).sleep()
			goal_msg = serial_msg()
			goal_msg.micro_id = send_msg.micro_id
			goal_msg.topic = "grip_feedback"
			goal_msg.msg = str(1)
			self._pub.publish(goal_msg)


if __name__ == '__main__':
	rospy.init_node('Psuedo_Serial_Com')
	sc = SerialCom()