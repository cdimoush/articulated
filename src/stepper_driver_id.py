#! /usr/bin/env python

'''
Stepper Driver Identifier

Each stepper motor is controlled/paired with an arduino nano. The serial ports for each nano constantly with plug configuration/reboot.
For this reason each stepper is given an ID number (ID list included somewhere in package). When pinged with 0, the arduino will return
its unique ID number. Once the ID is recieved the arduino's port name will be uploaded to the parameter server under /articulated/stepper/ID#.
This should be the first script run as the port names are needed by the stepper driver server.

'''

import os
import rospy
import serial

def read_arduino(ser):
	#number of read attempts. Note each readline request will wait 1 second
	attempts = 10
	
	for i in range(attempts):
		try:
			raw = ser.readline().decode().strip('\r\n')
			value = int(raw)
			return True, value
		except ValueError:
			rospy.loginfo('Waiting for ID')

	return False, 0

def main():
	#Name of serial device folder and potential names given to arduinos
	device_folder_prefix = '/dev/tty'
	names = ['ACM0', 'ACM1', 'ACM2', 'USB0', 'USB1', 'USB2', 'USB3', 'USB4', 'USB5']
	rate=9600

	for i in names:
		#If device name exists it will be pinged for an ID. 
		#NOTE: if an arduino is plugged in that is not apart of articulated robt it will recieved '0'
		if os.path.exists(device_folder_prefix + i):
			rospy.loginfo('Trying device %s', i)
			ser = serial.Serial(port=device_folder_prefix + i, baudrate=rate, timeout=1)
			#Readline is called after serial init. Don't know why but can't read in future when this line isn't included
			empty = ser.readline() 
			rospy.Rate(1).sleep() #important

			msg_out = str(0).encode('ascii')
			ser.write(msg_out)
			
			#Tuple is returned. msg_in[0]: boolean for successful read. msg_in[i]: arduino ID
			msg_in = read_arduino(ser)

			if msg_in[0]:
				rospy.loginfo('%s ID: %s', i, str(msg_in[1]))
				#set param as full serial port name
				param_name = 'articulated/stepper/' + str(msg_in[1])
				rospy.set_param(param_name, device_folder_prefix + i)
			else:
				rospy.logerr('No ID recieved for %s', i)	



if __name__ == '__main__':
	rospy.init_node('Stepper_ID')
	rospy.loginfo('Starting stepper_driver_id.py')
	main()