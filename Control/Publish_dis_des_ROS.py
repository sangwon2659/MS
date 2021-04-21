#!/usr/bin/env python
import rospy
import getch
from msgpkg.msg import Distance

def talker():
        pub = rospy.Publisher('dis_des', Distance, queue_size=10)
        rospy.init_node('Dis_Send', anonymous = True)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
                x = input("Distance : ")
                rospy.loginfo(str(x))
                pub.publish(x)

if __name__ == '__main__':
        try:
                talker()
        except rospy.ROSInterruptException:
                pass

