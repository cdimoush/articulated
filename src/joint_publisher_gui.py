#!/usr/bin/env python

from Tkinter import *
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import time
import math

class JointGui(object):
	scale = 2500
	def start(self):
		self.root = Tk()
		self.build()

		self.jpub = rospy.Publisher('articulated/joint_state_goal', Float64MultiArray, queue_size=2)

		while not rospy.is_shutdown():
			self.root.mainloop()

	def build(self):
		self.s1 = Scale(self.root, from_=0, to=120, orient=HORIZONTAL)
		self.s1.set(90)
		bl1 = Button(self.root, text='<', command=self.left1)
		br1 = Button(self.root, text='>', command=self.right1)
		bl1.grid(row=0, column=0)
		self.s1.grid(row=0, column=1)
		br1.grid(row=0, column=2)

		self.s2 = Scale(self.root, from_=0, to=180, orient=HORIZONTAL)
		self.s2.set(90)
		bl2 = Button(self.root, text='<', command=self.left2)
		br2 = Button(self.root, text='>', command=self.right2)
		bl2.grid(row=1, column=0)
		self.s2.grid(row=1, column=1)
		br2.grid(row=1, column=2)

		bpub = Button(self.root, text='Publish', command=self.publishJoints)
		bcal = Button(self.root, text='Calibrate', command=self.calibrate)
		bpub.grid(row=4, column=0)
		bcal.grid(row=4, column=1)
		
	def left1(self):
		i = self.s1.get()
		if (i-1) >= 0:
			self.s1.set(i-1)  

	def right1(self):
		i = self.s1.get()
		if (i+1) <= 120:
			self.s1.set(i+1)

	def left2(self):
		i = self.s2.get()
		if (i-1) >= 0:
			self.s2.set(i-1) 

	def right2(self):
		i = self.s2.get()
		if (i+1) <= 180:
			self.s2.set(i+1)

	def publishJoints(self):
		array = [self.s1.get(), self.s2.get()]
		msg_out = Float64MultiArray(data=array)
		self.jpub.publish(msg_out)

	def calibrate(self):
		self.s1.set(90)
		self.s2.set(90) 


if __name__ == '__main__':
	rospy.init_node('Joint_Publisher_GUI', anonymous=True)
	gui = JointGui()
	gui.start()

