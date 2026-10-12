#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class GoalRelativePublisher:
    def __init__(self):
        rospy.init_node('goal_relative_publisher')

        self.camera_frame = rospy.get_param('~camera_frame', '/active_neck_color_optical_frame')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.unity_topic = rospy.get_param('~unity_topic', '/unity_ball_relative_position')
        self.rate_hz = rospy.get_param('~rate', 30)

        self.pub = rospy.Publisher(self.unity_topic, Point, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback)

        self.listener = tf.TransformListener()

        self.have_goal = False
        self.goal_point = None
        self.goal_frame = "map"

        rospy.loginfo("Publishing last goal relative to %s on %s",
                      self.camera_frame, self.unity_topic)

        self.loop()

    def goal_callback(self, msg: PoseStamped):
        self.goal_point = np.array([msg.pose.position.x,
                                    msg.pose.position.y,
                                    msg.pose.position.z])
        self.goal_frame = msg.header.frame_id if msg.header.frame_id else "map"
        self.have_goal = True

    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.have_goal:
                try:
                    # lookup transform from camera->goal_frame
                    (trans, rot) = self.listener.lookupTransform(self.camera_frame,
                                                                 self.goal_frame,
                                                                 rospy.Time(0))
                    # apply transform: goal in goal_frame → camera_frame
                    mat = tf.transformations.quaternion_matrix(rot)
                    mat[0:3, 3] = trans
                    goal_h = np.append(self.goal_point, 1.0)
                    goal_in_cam = np.dot(mat, goal_h)[0:3]

                    # Convert to Unity coords
                    unity_pos = Point()
                    unity_pos.x = goal_in_cam[0]
                    unity_pos.y = -goal_in_cam[1]
                    unity_pos.z = goal_in_cam[2]

                    self.pub.publish(unity_pos)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn_throttle(5.0, "TF lookup failed: %s" % e)

            rate.sleep()

if __name__ == '__main__':
    try:
        GoalRelativePublisher()
    except rospy.ROSInterruptException:
        pass

