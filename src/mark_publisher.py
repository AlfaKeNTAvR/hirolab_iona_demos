#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("mark_publisher_node", anonymous=True)
    
    # Create the publisher but don't publish
    pub = rospy.Publisher("/mark", String, queue_size=10)
    
    rospy.loginfo("Publisher to /mark registered, but no messages will be published.")
    
    # Keep node alive so publisher stays registered
    rospy.spin()

if __name__ == "__main__":
    main()
