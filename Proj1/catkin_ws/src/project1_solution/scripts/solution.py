#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

def call_back(msg):
    add = msg.a + msg.b
    pub.publish(add)

rospy.init_node('add_ints')
sub = rospy.Subscriber('/two_ints', TwoInts, call_back)
pub = rospy.Publisher('sum', Int16, queue_size=10)
rospy.spin()
