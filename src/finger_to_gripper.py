#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float32

class FingerToGripper:
    def __init__(self):
        rospy.init_node('finger_to_gripper_node', anonymous=True)

        self.pub = rospy.Publisher('/left_arm/gripper_value', Float32, queue_size=10)
        self.sub = rospy.Subscriber('/fingersPos1', Float32MultiArray, self.callback)

    def callback(self, msg):
        if len(msg.data) > 0:
            value = msg.data[0]
            out_msg = Float32()
            out_msg.data = value
            self.pub.publish(out_msg)
        else:
            rospy.logwarn("Received empty data array on /fingersPos1")

if __name__ == '__main__':
    try:
        node = FingerToGripper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
