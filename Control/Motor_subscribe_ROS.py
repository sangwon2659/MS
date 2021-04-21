#!/usr/bin/env python

# Subscribes to the desired distance
# Controls the motor with respect to the subscribing data

import rospy
import math
import time
from dynamixel_sdk import *
from msgpkg.msg import Distance

# Declaring variables
M_PI = 3.14159265358979323846
init_pos = 0
dxl_present_position=0
dis_des = 0

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
DEVICENAME                  = '/dev/ttyUSB1'

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def callback(data):
    global dis_des
    global init_pos
    dis_des = int(data.Data/M_PI*2048+init_pos)

def dxl():
    global dis_des
    global dxl_present_position
    global init_pos
    rospy.init_node('Motor', anonymous=True)
    rate = rospy.Rate(200)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1)

    init_pos = dxl_present_position
    dis_des = init_pos

    # Declaring time for loop time and time tracking
    t0 = time.time()

    while not rospy.is_shutdown():
        cur = time.time()
        t_cur = cur-t0

        # Subscribing desired distance data     
        rospy.Subscriber("dis_des", Distance, callback)

        # Acquiring current information of the motor
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

        # Printing the log of the current information of the motor
        print('T: %3.3f \t Goal: %4d \t Pre: %4d \t Err: %2d \t'
        %(round(t_cur,3), dis_des, dxl_present_position, dis_des-dxl_present_position))

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dis_des)

        # Safety net
        '''
        if(abs(dis_des)>500):
                dxl_goal_position = 2048
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
                break
        '''

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl()
    except rospy.ROSInterruptException:
        pass

