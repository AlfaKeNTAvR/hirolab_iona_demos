#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from threading import Lock

from kinova_positional_control.srv import GripperPosition

class GripperController(object):
    def __init__(self):
        # Parameters
        self.topic = rospy.get_param("~input_topic", "/left_arm/gripper_value")
        self.service_name = rospy.get_param(
            "~service_name", "/left_arm/gripper_control/position"
        )

        self.open_position = rospy.get_param("~open_position", 1.0)
        self.close_position = rospy.get_param("~close_position", 0.0)

        self.upper_threshold = rospy.get_param("~upper_threshold", 0.7)
        self.lower_threshold = rospy.get_param("~lower_threshold", 0.3)

        if self.lower_threshold >= self.upper_threshold:
            rospy.logwarn("Lower threshold >= upper threshold. Adjusting.")
            self.lower_threshold = self.upper_threshold - 0.1

        self.state = None  # None, "open", "closed"
        self.lock = Lock()

        # Wait for service
        rospy.loginfo("Waiting for service %s..." % self.service_name)
        rospy.wait_for_service(self.service_name)
        self.gripper_srv = rospy.ServiceProxy(self.service_name, GripperPosition)
        rospy.loginfo("Connected to gripper service.")

        # Subscriber
        self.sub = rospy.Subscriber(self.topic, Float32, self.callback)

    def call_gripper(self, position):
        try:
            resp = self.gripper_srv(position=position)
            rospy.loginfo("Gripper command sent: %.2f (state=%s)" %
                          (position, resp.state))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def callback(self, msg):
        val = msg.data

        with self.lock:
            # Hysteresis logic
            if self.state != "open" and val >= self.upper_threshold:
                rospy.loginfo("Value %.3f >= upper_threshold %.3f → OPEN",
                              val, self.upper_threshold)
                self.call_gripper(self.open_position)
                self.state = "open"

            elif self.state != "closed" and val <= self.lower_threshold:
                rospy.loginfo("Value %.3f <= lower_threshold %.3f → CLOSE",
                              val, self.lower_threshold)
                self.call_gripper(self.close_position)
                self.state = "closed"


if __name__ == "__main__":
    rospy.init_node("gripper_controller")

    try:
        controller = GripperController()
        rospy.loginfo("Gripper controller node started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
