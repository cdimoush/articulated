#!/usr/bin/env python
import rospy
import serial

from articulated.srv import StepperDriver

'''
Stepper Driver Server

The service takes the stepper ID and desired steps as service request parameters.
A serial (ser) is made for each request and mapped to the desired port using the
stepper id parameter. The desired steps are then sent to an arduino nano and the
service waits for the arduino to send complete
'''


class StepperDriverServer:
	def __init__(self):
		rospy.init_node('stepper_driver_server')
		self.init_serial_coms()
		s = rospy.Service('stepper_driver_service', StepperDriver, self.handle_request)
		rospy.spin()

	def init_serial_coms(self):
		rospy.loginfo('Creating serial coms')
		
		rate = 9600
		number_of_ports = 0
		port_param_prefix = 'articulated/stepper/'
		self._ser_coms = []
		#port_name = port_param_prefix + str(0)
		#port_address = rospy.get_param(port_name)
		
		#self._ser_coms = [serial.Serial(port=port_address, baudrate=rate, timeout=1)]

		
		for i in range(2):
			port_name = port_param_prefix + str(i)
			if rospy.has_param(port_name):
				port_address = rospy.get_param(port_name)
				self._ser_coms.append(serial.Serial(port=port_address, baudrate=rate, timeout=1))
				rospy.loginfo('Serial com created for stepper %s', str(i))
				#Readline is called after serial init. Don't know why but can't read in future when this line isn't included
				empty = self._ser_coms[i].readline() 
				#rospy.Rate(1).sleep()

	def handle_request(self, req):
		success = False
		step_counter = 0
		
		#param_name = 'articulated/stepper/' + str(req.stepper_id)
		#port_name = rospy.get_param(param_name)

		'''
		#create the serial object
		rospy.loginfo('Creating serial com')
		self._ser = serial.Serial(port=port_name, baudrate=rate, timeout=1)
		#Readline is called after serial init. Don't know why but can't read in future when this line isn't included
		empty = self._ser.readline() 
		#rospy.Rate(1).sleep()
		'''

		rospy.loginfo('Steps sent: %s', str(req.steps))
		msg_out = str(req.steps).encode('ascii')
		#empty = self._ser_coms[req.stepper_id].readline() 
		self._ser_coms[req.stepper_id].write(msg_out)

		#rospy.Rate(1).sleep()
		#self._ser.flushInput()

		while not success:
			msg_in = self.read_arduino(req.stepper_id)
			if msg_in[0]: #a msg recieved
				step_counter += 1
				if msg_in[1] == 1:
					rospy.loginfo('Steps: %s', step_counter)
				if msg_in[1] == 0:
					rospy.loginfo('Complete')
					success = True

			elif not msg_in[0]:
				return False, 0

		#rospy.Rate(1).sleep()
		return True, 0

	def read_arduino(self, s_id):
		#number of read attempts. Note each readline request will wait 1 second
		attempts = 3

		for i in range(attempts):
			try:
				raw = self._ser_coms[s_id].readline().decode().strip('\r\n')
				value = int(raw)
				return True, value
			except ValueError:
				pass

		return False, 0


if __name__ == "__main__":
    server = StepperDriverServer()


