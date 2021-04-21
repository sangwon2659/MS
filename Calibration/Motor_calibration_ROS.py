#!/usr/bin/env python
# To be used for calibrating the FSS sensors
# Controls the motor with a sine sweep

import rospy
import math
import time
from dynamixel_sdk import *

# Declaring variables
M_PI = 3.14159265358979323846
looptime = 1./200
dxl_present_position=0

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PROFILE_VELOCITY    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0             

# Default setting
DXL_ID                      = 4                   # Dynamixel#1 ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB2'    

portHandler = PortHandler(DEVICENAME)

packetHandler = PacketHandler(PROTOCOL_VERSION)

def dxl():    
    global dxl_present_position
    portHandler.openPort()
    # Variable can be brought into function and used
    # When the same variable declared in the function that variable is different from the one outside
    # They are the same only when the variable is declared as a global variable
    portHandler.setBaudRate(BAUDRATE)

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1)
    dxl_goal_position = 2048
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

    init_pos = dxl_present_position
    
    # Declaring time for loop time and time tracking
    t_pre = time.time()
    t0 = time.time()

    while not rospy.is_shutdown():
	# Declaring current time
	cur = time.time()

	# Setting loop time
	if (cur - t_pre)>looptime:
	    t_cur = cur-t0
	    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

	    # Sinusoidal Input
	    mag = 0.12
	    if t_cur<5 :
		theta_des = 0
	    elif t_cur< 125:
		theta_des = -mag*math.sin(M_PI*(t_cur-5)/20)
		if theta_des >0:
		    theta_des = 0
	    else :
		theta_des = 0
	    
	    dxl_goal_position = int(theta_des/M_PI*2048+init_pos)
	    
	    print('T: %3.3f \t Goal: %4d \t Pre: %4d \t Err: %2d \t ang_d: %2.3f'
	%(round(t_cur,3),dxl_goal_position, dxl_present_position, dxl_goal_position-dxl_present_position, theta_des))	    
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
	    
	    t_pre = cur

	    # Safety net
	    if(abs(theta_des)>1.3):
		dxl_goal_position = 2048
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
            	dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
		break

if __name__ == '__main__':
    try:
        dxl()
    except rospy.ROSInterruptException:
        pass


