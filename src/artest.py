#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point

def publish_relative_position():
    rospy.init_node('ball_relative_publisher')

    # TF listener
    listener = tf.TransformListener()
    rate = rospy.Rate(30)  # 30 Hz

    # Publisher for Unity
    pub = rospy.Publisher('/unity_ball_relative_position', Point, queue_size=1)

    while not rospy.is_shutdown():
        try:
            # Get transform from camera to ball
            (trans, rot) = listener.lookupTransform('/active_neck_color_optical_frame', 'right_arm/tool_frame', rospy.Time(0))

            # ROS camera frame: x-right, y-down, z-forward
            # Unity frame: x-right, y-up, z-forward
            unity_pos = Point()
            unity_pos.x = trans[0]       # x-right is same
            unity_pos.y = -trans[1]      # invert y
            unity_pos.z = trans[2]       # z-forward same

            pub.publish(unity_pos)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()

if __name__ == '__main__':
    publish_relative_position()
