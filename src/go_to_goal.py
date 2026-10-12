#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Twist

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

rospy.init_node("rotate_and_report_fixed")

# Params
base_frame = rospy.get_param("~base_frame", "base_link")
forward_axis = rospy.get_param("~forward_axis", "x")  # options: "x","y","-x","-y"
yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.04)  # rad
k_ang = rospy.get_param("~k_ang", 0.7)
max_ang = rospy.get_param("~max_ang", 0.8)  # rad/s
min_distance_to_report = rospy.get_param("~min_distance", 0.01)
rate_hz = rospy.get_param("~rate", 20.0)

# Map forward_axis to angle offset (angle of forward axis measured from +X)
if forward_axis == "x":
    forward_angle = 0.0
elif forward_axis == "y":
    forward_angle = math.pi / 2.0
elif forward_axis == "-x":
    forward_angle = math.pi
elif forward_axis == "-y":
    forward_angle = -math.pi / 2.0
else:
    rospy.logwarn("Unknown forward_axis '%s', defaulting to 'x'." % forward_axis)
    forward_angle = 0.0

tf_listener = tf.TransformListener()
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
goal_pose = None

def goal_cb(msg):
    global goal_pose
    goal_pose = msg
    rospy.loginfo("New goal in frame '%s'." % msg.header.frame_id)

rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb)

rate = rospy.Rate(rate_hz)
rospy.loginfo("rotate_and_report_fixed: base_frame=%s forward_axis=%s" % (base_frame, forward_axis))

while not rospy.is_shutdown():
    if not goal_pose:
        rate.sleep()
        continue

    try:
        # transform goal into base_link frame
        goal_in_base = tf_listener.transformPose(base_frame, goal_pose)
        x = goal_in_base.pose.position.x
        y = goal_in_base.pose.position.y
        dist = math.hypot(x, y)

        # angle relative to +X axis, then subtract forward_angle to get error for robot forward
        angle_to_goal = math.atan2(y, x) - forward_angle
        angle_error = normalize_angle(angle_to_goal)

        # print distance once (or throttle)
        if dist > min_distance_to_report:
            rospy.loginfo_throttle(2.0, "Distance to goal (would drive): %.3f m" % dist)
        else:
            rospy.loginfo("Goal very close: %.3f m (treating as reached)" % dist)

        cmd = Twist()
        # rotate only, no forward motion
        if abs(angle_error) > yaw_tolerance:
            # P-control with cap, using sign of error
            ang = k_ang * angle_error
            if ang > max_ang: ang = max_ang
            if ang < -max_ang: ang = -max_ang
            cmd.angular.z = ang
            cmd.linear.x = 0.0
        else:
            # Facing goal — stop and clear goal (test mode)
            rospy.loginfo("Facing goal direction. (Test mode: not driving forward). Distance: %.3f m" % dist)
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            goal_pose = None

        cmd_pub.publish(cmd)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn_throttle(5.0, "TF transform not ready: %s" % str(e))

    rate.sleep()
