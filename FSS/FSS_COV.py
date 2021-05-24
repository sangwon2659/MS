#!/usr/bin/env python

import rospy
import serial
import struct
import numpy as np
import threading
from gripperslip.msg import Cov

# For 5 FSS Sensors
n_ch = 5
# Declaring an array so store data and display it later
data = np.array((0,0,0,0,0))
#value = data
#alpha = 0.0
cov = 0

# Declaring the USB port for which the arduino is connected
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)

def thread_run():
	global cov, pub
	# Publishing the value and torque
	pub.publish(cov)
	# Displaying the value on the log
	#rospy.loginfo(value)
	threading.Timer(0.00000001, thread_run).start()

def talker():
	global cov, pub
	
	# Initializing the node with the name 'FSS'
	rospy.init_node('FSS_cov', anonymous=True)
	# Declaring publisher with topic name 'FSS' and message name 'FSS'
	pub = rospy.Publisher('FSS_cov', Cov, queue_size=1)

	ser.reset_input_buffer()
	thread_run()

	n=0

	while not rospy.is_shutdown():
		# Reading the data from the arduino
		response = ser.readline()
		# 4 bytes of data for each channel from the arduino + 1 byte of spacing
		# Needs more number of bytes compared to the ADC one
		if response.__len__() == n_ch*4+1:
			# A method of unpacking the data
			# There are 3 Hs since the number of channels is 3
			# '<' for starting and l for 4 bytes of long type data
			# Only considering upto the second last byte (response[:-1])
			# Torque data using the SEA not included in this file
			if(n==0):
				data_matrix = np.zeros((5,5))
				data_previous = np.array(struct.unpack('<lllll', response[:-1]))
				n+=1
			else:
				data = np.array(struct.unpack('<lllll', response[:-1]))
				data_difference = data-data_previous
				data_matrix = np.vstack((data_difference,data_matrix))
				data_matrix = np.delete(data_matrix, 5, axis=0)
				data_previous = data
				cov_matrix = np.dot(np.transpose(data_matrix),data_matrix)
				cov = np.sum(cov_matrix)-np.trace(cov_matrix)
				rospy.loginfo(cov)
				#value = (alpha*value+(1-alpha)*data).astype('int32')


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass








