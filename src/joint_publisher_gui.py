#!/usr/bin/env python

from Tkinter import *
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

import time
import math

class JointGui(object):
	scale = 2500
	def start(self):
		self.root = Tk()
		self.build()

		self.jpub = rospy.Publisher('articulated/joint_state_goal', Float64MultiArray, queue_size=2)
		ee_sub = rospy.Subscriber('articulated/ee_pos', Pose, self.eeCallback)

		while not rospy.is_shutdown():
			self.root.mainloop()

	def build(self):
		l1 = Label(self.root, text='Stepper 1')
		self.s1 = Scale(self.root, from_=0, to=120, orient=HORIZONTAL)
		self.s1.set(90)
		bl1 = Button(self.root, text='<', command=self.left1)
		br1 = Button(self.root, text='>', command=self.right1)
		l1.grid(row=0, column=0)
		bl1.grid(row=0, column=1, sticky=E)
		self.s1.grid(row=0, column=2)
		br1.grid(row=0, column=3, sticky=W)

		l2 = Label(self.root, text='Stepper 2')
		self.s2 = Scale(self.root, from_=0, to=180, orient=HORIZONTAL)
		self.s2.set(90)
		bl2 = Button(self.root, text='<', command=self.left2)
		br2 = Button(self.root, text='>', command=self.right2)
		l2.grid(row=1, column=0)
		bl2.grid(row=1, column=1, sticky=E)
		self.s2.grid(row=1, column=2)
		br2.grid(row=1, column=3, sticky=W)

		bpub = Button(self.root, text='Publish', command=self.publishJoints)
		bcal = Button(self.root, text='Calibrate', command=self.calibrate)
		bpub.grid(row=0, column=5, sticky=E)
		bcal.grid(row=1, column=5, sticky=E)

		lee = Label(self.root, text="End Effector")
		leex= Label(self.root, text="X")
		leey= Label(self.root, text="Y")
		self.x = Label(self.root, text='0')
		self.y = Label(self.root, text='0')
		lee.grid(row=3, column=0)
		leex.grid(row=4, column=0, sticky=W)
		leey.grid(row=4, column=1, sticky=W)
		self.x.grid(row=5, column=0, sticky=W)
		self.y.grid(row=5, column=1, sticky=W)
		
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

	def eeCallback(self, ee_pos):
		self.x.configure(text=str(round(ee_pos.position.x,3)))
		self.y.configure(text=str(round(ee_pos.position.y,3)))
		

		


if __name__ == '__main__':
	rospy.init_node('Joint_Publisher_GUI', anonymous=True)
	gui = JointGui()
	gui.start()

