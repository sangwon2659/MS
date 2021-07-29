#!/usr/bin/env python

import rospy
import serial
import struct
import numpy as np
import threading
from keras.models import load_model
from numpy import append
from scipy import cos, sin
from scipy.fftpack import fft, fftfreq
from gripperslip.msg import FSS
from gripperslip.msg import DataVector
from gripperslip.msg import SlipDetection

# Using 2 of 5-Channel FSS Sensors
n_ch = 5

# Declaring an array so store data and display it later
data = np.array((0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
data_sum = np.array((0))
value = data
alpha = 0.0
thread_ = 0.01
n = 10

Data_Vector = np.array((0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
Slip_Vector = Data_Vector

Slip_NoSlip = 0.0

# Declaring variables for Covariance
Cov = 0
Data_Matrix = np.zeros((10,10))
Cov_Matrix = np.zeros((10,10))

# Declaring variables for FFT
FFT_Vector = np.array((0))
n_sample = 20

# Declaring the USB port for which the arduino is connected
ser = serial.Serial(port = '/dev/ttyUSB2', baudrate=115200)
ser_ = serial.Serial(port = '/dev/ttyUSB1', baudrate=115200)

def thread_run():
	global pub, pub2, pub3, FFT, value, Data_Vector, Slip_Vector, Slip_NoSlip
	# Publishing the FFS and FFT data
	pub.publish(Slip_NoSlip)
	pub2.publish(Slip_Vector)
	#pub3.publish(FSS_value)
	# Displaying the data on the log
	#rospy.loginfo(Slip_NoSlip)
	#rospy.loginfo(Slip_Vector)
	#rospy.loginfo(FSS_value)
	threading.Timer(thread_, thread_run).start()

def talker():
	global pub, pub2, pub3, data_sum, value, data, Cov, Data_Matrix, Cov_Matrix, FFT_Vector, Data_Vector, Slip_Vector, Slip_NoSlip
	
	# Initializing the node with the name 'talker'
	rospy.init_node('SlipDetection', anonymous=True)
	# Declaring publisher with topic name 'FFT' and message name 'FFT'
	pub = rospy.Publisher('Slip_NoSlip', SlipDetection, queue_size=1)
	pub2 = rospy.Publisher('Data_Vector', DataVector, queue_size=1)
	#pub3 = rospy.Publisher('FSS', FSS, queue_size= 1)
	
	# Loading model
	MLP_Model = load_model('MLP_Model.h5')
	ver = 0
	reference = [55,56,57,58,59,60,61,62,63,64,65]

	ser.reset_input_buffer()
	ser_.reset_input_buffer()
	thread_run()

	while not rospy.is_shutdown():
		# Reading the data from the arduino
		response = ser.readline()
		response_ = ser_.readline()

		# 5 bytes of data for each channel from the arduino + 1 byte of spacing
		# Needs more number of bytes compared to the ADC one
		if response.__len__() == n_ch*4+1 and response_.__len__() == n_ch*4+1:
			# A method of unpacking the data
			# There are 5 ls since the number of channels is 5
			# '<' for starting and l for n_ch bytes of long type data
			# Only considering upto the second last byte (response[:-1])
			data[0:5] = np.array(struct.unpack('<lllll', response[:-1]))
			data[5:10] = np.array(struct.unpack('<lllll', response_[:-1]))
			
			# Normalization with 8000000 (an approx of 2^23)
			data = data/8000000	
			
			FSS_value = (alpha*value+(1-alpha)*data).astype('int32')
						
			# Computing the Cov and FFT
			temp = data.sum()
			data_sum = np.append(data_sum, temp)
			if ver==0:
				data_previous = data
				ver+=1			

			if len(data_sum) > n_sample:
				# Deleting earliest one in queue
				data_sum = np.delete(data_sum, [0])
				
				# Computing Covariance
				data_diff = data-data_previous
				Data_Matrix = np.vstack((data_diff, Data_Matrix))
				Data_Matrix = np.delete(Data_Matrix, 10, axis=0)
				data_previous[:] = data[:]
				Cov_Matrix = np.dot(np.transpose(Data_Matrix), Data_Matrix)
				Cov = np.sum(Cov_Matrix)-np.trace(Cov_Matrix)

				# Computing FFT
				FFT_Vector = np.fft.fft(data_sum)
				FFT_Vector = FFT_Vector[range(int(len(data_sum)/2))]	
				FFT_Vector = abs(FFT_Vector)		

				# Combining Cov and FFT	
				Data_Vector[0:10] = FFT_Vector
				Data_Vector[10] = Cov
				
				Slip_Vector = np.hstack((Data_Vector, Slip_Vector))
				if len(Slip_Vector)==66:
					Slip_Vector = np.delete(Slip_Vector, reference)
					Slip_Vector_Reshaped = Slip_Vector.reshape(1,55)
					Slip_NoSlip = MLP_Model.predict(Slip_Vector_Reshaped)	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
