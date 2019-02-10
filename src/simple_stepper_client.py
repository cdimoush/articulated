#!/usr/bin/env python
import rospy
from articulated.srv import StepperDriver

def stepper_driver_client(stepper_id, steps):
	rospy.wait_for_service('stepper_driver')
	try:
		stepper_request = rospy.ServiceProxy('stepper_driver_service', StepperDriver)
		stepper_response = stepper_request(stepper_id, steps)
		return stepper_response.result

	except rospy.ServiceException, e:
		print "Client Call Failure"

if __name__ == "__main__":
	print "Service result = %s"%(stepper_driver_client(1, 100))