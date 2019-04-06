#! /usr/bin/env python

import os
import rospy
import time
import serial

from articulated.msg import serial_msg

class SerialCom:
	_stepper_states = [0, 0]
	_stepper_goals = [0, 0]
	def __init__(self):
		#Name of serial device folder and potential names given to arduinos
		rospy.loginfo('%s: Initializing', rospy.get_name())

		self._pub = rospy.Publisher('articulated/serial/receive', serial_msg, queue_size=10)
		rospy.Subscriber('articulated/serial/send', serial_msg, self.sendMsgCallback)
		self.main()

	def main(self):
		while not rospy.is_shutdown():
			if self._stepper_states != self._stepper_goals:
				for i in range(2):
					x = self._stepper_goals[i] - self._stepper_states[i]
					if x != 0: 
						#Shorter Delay for stepper 1
						if i == 0:
							rospy.Rate(450).sleep()
						if i == 1:
							rospy.Rate(25).sleep()
						if x > 0:
							self._stepper_states[i] += 1
						if x < 0:
							self._stepper_states[i] -= 1

						receive_msg = serial_msg()
						receive_msg.micro_id = i
						receive_msg.topic = "step_feedback"
						receive_msg.msg = str(self._stepper_states[i])
						self._pub.publish(receive_msg)
					
			else:
				for i in range(2):
					if self._stepper_states[i] != 0:
						self._stepper_states[i] = 0
						self._stepper_goals[i] = 0
				rospy.Rate(500).sleep()

	def sendMsgCallback(self, send_msg):
		if send_msg.topic == "set_step_pos":
			self._stepper_goals[send_msg.micro_id] = int(send_msg.msg)

if __name__ == '__main__':
	rospy.init_node('Psuedo_Serial_Com')
	sc = SerialCom()