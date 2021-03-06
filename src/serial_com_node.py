#! /usr/bin/env python

import os
import rospy
import time
import serial

from articulated.msg import serial_msg

class SimpleSerial:
	_read_in_progress = False
	_read_mark_start = "<"
	_read_mark_end = ">"
	_input_buffer = ""
	_id = 0

	def __init__(self, port, rate, cb):
		self.subscriber_cb = cb
		self._port = port
		self._ser = serial.Serial(port=port, baudrate=rate, timeout=1)
		empty = self._ser.readline()
		self._ser.flushOutput()
		self._ser.flushInput()
		time.sleep(1.5)

	def setID(self, micro_id):
		self._id = micro_id
		
	def publish(self, topic, msg):
		msg = "<" + topic + ":" + str(msg) + ">"
		self._ser.write(msg)

	def parseDataIn(self, s):
		split = s.index(":")
		topic = s[0:split]
		msg = s[split + 1:]

		self.subscriber_cb(self._port, self._id, topic, msg)

	def spin(self):
		if self._ser.inWaiting() != 0:
			x = self._ser.read().decode("utf-8")
			if (x == self._read_mark_end):
				
				self.parseDataIn(self._input_buffer)

				self._input_buffer = ""
				self._read_in_progress = False
				
			elif(self._read_in_progress):
				self._input_buffer += x
			
			elif(x == self._read_mark_start):
				self._read_in_progress = True

class SerialCom:
	_port_id_in_progress = True
	_port_id_attempts = []
	_ser_com_list = [None] * 3
	def __init__(self):
		#Name of serial device folder and potential names given to arduinos
		rospy.loginfo('%s: Initializing', rospy.get_name())

		self._pub = rospy.Publisher('articulated/serial/receive', serial_msg, queue_size=10)
		rospy.Subscriber('articulated/serial/send', serial_msg, self.sendMsgCallback)

		device_folder_prefix = '/dev/tty'
		names = ['ACM0', 'ACM1', 'ACM2', 'USB0', 'USB1', 'USB2', 'USB3', 'USB4', 'USB5']
		rate=9600

		for i in names:
			#If device name exists it will be pinged for an ID. 
			#NOTE: if an arduino is plugged in that is not apart of articulated robt it will still receive a msg
			if os.path.exists(device_folder_prefix + i):
				port = device_folder_prefix + i
				ser = SimpleSerial(port, rate, self.arduinoCb)
				ser.publish("stepper_id", "")
				
				p_id_tuple = (port, ser)
				self._port_id_attempts.append(p_id_tuple)
		
		self.main()

	def main(self):
		#NOTE: Program assumes all arduinos will respond to the 'stepper_id' call
		#This could lead to issues when more devices are connected to usb
		rospy.loginfo('%s: Get IDs', rospy.get_name())
		rospy.loginfo('%s: Trying %s devices', rospy.get_name(), str(len(self._port_id_attempts)))
		while self._port_id_in_progress:
			try:
				for i in self._port_id_attempts:
					i[1].spin()
				if not self._port_id_attempts:
					self._port_id_in_progress = False
				rospy.Rate(100).sleep()
			except rospy.ROSInterruptException:
				break

		rospy.loginfo('%s: ID complete', rospy.get_name())

		while not rospy.is_shutdown():
			for i in self._ser_com_list:
				if i is not None:
					i.spin()
			rospy.Rate(100).sleep()


	def arduinoCb(self, port, m_id, topic, msg):
		if topic == "stepper_id":
			self.setStepperIdParams(port, topic, msg);
		else:
			receive_msg = serial_msg()
			receive_msg.micro_id = m_id
			receive_msg.topic = topic
			receive_msg.msg = msg
			self._pub.publish(receive_msg)

	def setStepperIdParams(self, port, topic, msg):
		if self._port_id_attempts:
			micro_id = int(msg)
			for i in self._port_id_attempts:
				if port == i[0]:
					rospy.loginfo('%s: Stepper %s is on port %s', rospy.get_name(), msg, port)
					#param_name = 'articulated/stepper/' + str(msg) + '/port'
					#rospy.set_param(param_name, port)
					self._ser_com_list[micro_id] = i[1]
					i[1].setID(micro_id)

					self._port_id_attempts.remove(i)

	def sendMsgCallback(self, send_msg):
		self._ser_com_list[send_msg.micro_id].publish(send_msg.topic, send_msg.msg)


if __name__ == '__main__':
	rospy.init_node('Serial_Com')
	sc = SerialCom()