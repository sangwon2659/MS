#!/usr/bin/env python

import rospy
import serial
import struct
import numpy as np
import threading
from numpy import append
from scipy import cos, sin
from scipy.fftpack import fft, fftfreq
from msgpkg.msg import FSS

# For 5 FSS Sensors
n_ch = 5
# Declaring an array so store data and display it later
fft_theo = np.array((0))
data = np.array((0,0,0,0,0))
data_sum = np.array((0))
value = data
alpha = 0.0

#Declaring variables for FFT
n_sample = 1000
n_FFT = 1000
freqs = fftfreq(n_FFT)
mask = freqs > 0

# Declaring the USB port for which the arduino is connected
ser = serial.Serial(port = '/dev/ttyUSB2',baudrate=115200)

def thread_run():
	global value, pub, pub2, fft_theo
	# Publishing the FFS and FFT data
	pub.publish(value)
	#pub2.publish(fft_theo)
	# Displaying the value on the log
	#rospy.loginfo(value)
	threading.Timer(0.000001, thread_run).start()

def talker():
	global value, pub, pub2, data_sum, fft_theo
	
	# Initializing the node with the name 'talker'
	rospy.init_node('FSS', anonymous=True)
	# Declaring publisher with topic name 'FSS' and message name 'FSS'
	# Declaring publisher with topic name 'FFT' and message name 'FFT'
	pub = rospy.Publisher('FSS', FSS, queue_size=1)
	pub2 = rospy.Publisher('FFT', FSS, queue_size=1)

	ser.reset_input_buffer()
	thread_run()

	while not rospy.is_shutdown():
		# Reading the data from the arduino
		response = ser.readline()
		# 5 bytes of data for each channel from the arduino + 1 byte of spacing
		# Needs more number of bytes compared to the ADC one
		if response.__len__() == n_ch*4+1:
			# A method of unpacking the data
			# There are 5 ls since the number of channels is 5
			# '<' for starting and l for n_ch bytes of long type data
			# Only considering upto the second last byte (response[:-1])
			data = np.array(struct.unpack('<lllll', response[:-1]))
			# Computing the FFT
			'''
			temp = data.sum()
			data_sum = np.append(data_sum, temp)			
			if len(data_sum) > n_sample:
				data_sum = np.delete(data_sum, [0])
				fft_vals = fft(data_sum[0:n_sample])
				fft_norm = fft_vals*(1.0/n_FFT)
				fft_theo = 2.0*abs(fft_norm)
			# Computing the covariance
			'''
				
			value = (alpha*value+(1-alpha)*data).astype('int32')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
