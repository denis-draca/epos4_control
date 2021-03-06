#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16

pub = rospy.Publisher('/voltage/gui', Int16, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(100) # 10hz

x = 0

while not rospy.is_shutdown():
    rospy.loginfo(x)
    pub.publish(x)

    x += 1
    rate.sleep()
