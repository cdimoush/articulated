#!/usr/bin/env python

from Tkinter import *
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

import time
import math

class ArmGui(object):
	scale = 2500
	def start(self):
		self.root = Tk()
		self.build()

		self.stepper_pub = rospy.Publisher('articulated/set_stepper_angle', Float64MultiArray, queue_size=2)
		self.ik_pub = rospy.Publisher('articulated/pos_goal', Pose, queue_size=2)
		ee_sub = rospy.Subscriber('articulated/ee_pos', Pose, self.eeCallback)
		jt_sub = rospy.Subscriber('articulated/joint_state', JointState, self.jtCallback)

		while not rospy.is_shutdown():
			self.root.mainloop()


	def build(self):
		self.root.title('Articulated Proto GUI')

		l0 = Label(self.root, text='Stepper 0')
		self.s0 = Scale(self.root, from_=0, to=1200, orient=HORIZONTAL)
		self.s0.set(0)
		bl0 = Button(self.root, text='<', command=self.left0)
		br0 = Button(self.root, text='>', command=self.right0)
		l0.grid(row=0, column=0)
		bl0.grid(row=0, column=1, sticky=E)
		self.s0.grid(row=0, column=2)
		br0.grid(row=0, column=3, sticky=W)

		l1 = Label(self.root, text='Stepper 1')
		self.s1 = Scale(self.root, from_=0, to=120, orient=HORIZONTAL)
		self.s1.set(90)
		bl1 = Button(self.root, text='<', command=self.left1)
		br1 = Button(self.root, text='>', command=self.right1)
		l1.grid(row=1, column=0)
		bl1.grid(row=1, column=1, sticky=E)
		self.s1.grid(row=1, column=2)
		br1.grid(row=1, column=3, sticky=W)

		l2 = Label(self.root, text='Stepper 2')
		self.s2 = Scale(self.root, from_=-90, to=90, orient=HORIZONTAL)
		self.s2.set(0)
		bl2 = Button(self.root, text='<', command=self.left2)
		br2 = Button(self.root, text='>', command=self.right2)
		l2.grid(row=2, column=0)
		bl2.grid(row=2, column=1, sticky=E)
		self.s2.grid(row=2, column=2)
		br2.grid(row=2, column=3, sticky=W)

		bpub = Button(self.root, text='Publish', command=self.publishAngles)
		bcal = Button(self.root, text='Calibrate', command=self.calibrate)
		bpub.grid(row=0, column=5, sticky=E)
		bcal.grid(row=1, column=5, sticky=E)

		#IK Section
		self.root.grid_columnconfigure(6, minsize=25)
		l_ikx = Label(self.root, text="X")
		l_iky = Label(self.root, text="Y")
		l_ikz = Label(self.root, text="Z")
		l_ikx.grid(row=0, column=7)
		l_iky.grid(row=1, column=7)
		l_ikz.grid(row=2, column=7)

		self.e_ikx = Entry(self.root, text="X")
		self.e_iky = Entry(self.root, text="Y")
		self.e_ikz = Entry(self.root, text="Z")
		self.e_ikx.grid(row=0, column=8)
		self.e_iky.grid(row=1, column=8)
		self.e_ikz.grid(row=2, column=8)

		b_pub_ik = Button(self.root, text='Publish', command=self.publishIK)
		b_clear_ik = Button(self.root, text='Clear', command=self.clearIK)
		b_pub_ik.grid(row=0, column=9)
		b_clear_ik.grid(row=1, column=9)

		lee = Label(self.root, text="End Effector")
		leex= Label(self.root, text="X (m)")
		leey= Label(self.root, text="Y (m)")
		leez= Label(self.root, text="Z (m)")
		self.x = Label(self.root, text='0')
		self.y = Label(self.root, text='0')
		self.z = Label(self.root, text='0')
		lee.grid(row=4, column=0)
		leex.grid(row=5, column=0, sticky=W)
		leey.grid(row=5, column=1, sticky=W)
		leez.grid(row=5, column=2, sticky=E)
		self.x.grid(row=6, column=0, sticky=W)
		self.y.grid(row=6, column=1, sticky=W)
		self.z.grid(row=6, column=2, sticky=E)

		ljt = Label(self.root, text="Joints")
		ljt0= Label(self.root, text="Joint 0")
		ljt1= Label(self.root, text="Joint 1")
		ljt2= Label(self.root, text="Joint 2")
		self.jt0_pos = Label(self.root, text='0')
		self.jt1_pos = Label(self.root, text='0')
		self.jt2_pos = Label(self.root, text='0')
		ljt.grid(row=7, column=0, sticky=W)
		ljt0.grid(row=8, column=0, sticky=W)
		ljt1.grid(row=8, column=1, sticky=W)
		ljt2.grid(row=8, column=2, sticky=E)
		self.jt0_pos.grid(row=9, column=0, sticky=W)
		self.jt1_pos.grid(row=9, column=1, sticky=W)
		self.jt2_pos.grid(row=9, column=2, sticky=E)

		
		
	def left0(self):
		i = self.s0.get()
		if (i-1) >= 0:
			self.s0.set(i-1)  

	def right0(self):
		i = self.s0.get()
		if (i+1) <= 120:
			self.s0.set(i+1)

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
		if (i-1) >= -90:
			self.s2.set(i-1) 

	def right2(self):
		i = self.s2.get()
		if (i+1) <= 90:
			self.s2.set(i+1)

	def publishAngles(self):
		#Get current values of sliders
		array = [self.s0.get(), self.s1.get(), self.s2.get()]
		#Convert to Radians
		array = [i * (math.pi/180) for i in array]
		#Put into a float array and send
		msg_out = Float64MultiArray(data=array)
		self.stepper_pub.publish(msg_out)

	def calibrate(self):
		self.s1.set(90)
		self.s2.set(0) 
		self.s0.set(0) 

	def publishIK(self):
		p_goal = Pose()
		p_goal.position.x = float(self.e_ikx.get())
		p_goal.position.y = float(self.e_iky.get())
		p_goal.position.z = float(self.e_ikz.get())
		self.ik_pub.publish(p_goal)


	def clearIK(self):
		self.e_ikx.delete(0, 'end')
		self.e_iky.delete(0, 'end')
		self.e_ikz.delete(0, 'end')

	def eeCallback(self, ee_pos):
		self.x.configure(text=str(round(ee_pos.position.x,3)))
		self.y.configure(text=str(round(ee_pos.position.y,3)))
		self.z.configure(text=str(round(ee_pos.position.z,3)))

	def jtCallback(self, jt_pos):
		self.jt0_pos.configure(text=str(round(jt_pos.position[0] * (180/math.pi),3)))
		self.jt1_pos.configure(text=str(round(jt_pos.position[1] * (180/math.pi),3)))
		self.jt2_pos.configure(text=str(round(jt_pos.position[2] * (180/math.pi),3)))
		

if __name__ == '__main__':
	rospy.init_node('Articulated_GUI', anonymous=True)
	gui = ArmGui()
	gui.start()

