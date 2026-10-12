#!/usr/bin/env python3
"""
left_arm_gripper_value_control.py

Smooth continuous gripper control via Kortex speed mode.
Subscribes to a Float32 topic (0.0=open, 1.0=closed) and runs a
proportional velocity controller at ~40Hz — no Modbus, no bridge,
no trapezoidal stop-start between position commands.

HOW IT WORKS:
  Instead of sending position targets (which trigger full trapezoidal
  profiles and cause choppiness), this node sends SPEED commands every
  loop iteration. Speed = kp * (desired - actual), clamped to [-1, 1].
  The gripper tracks the input continuously and decelerates naturally
  as error shrinks. No discrete stop-go motion.

Parameters
----------
  ~input_topic   (str,   default: "/left_arm/gripper_value")
  ~robot_ip      (str,   default: "192.168.1.11")  [unused, kept for compat]
  ~rate          (float, default: 40.0)   Hz  (Kortex service ceiling)
  ~kp            (float, default: 3.0)    proportional gain
  ~deadband      (float, default: 0.005)  position error below which speed=0
  ~max_speed     (float, default: 1.0)    maximum speed command (0-1)
"""

import threading
import rospy
from std_msgs.msg import Float32
from kortex_driver.srv import (SendGripperCommand, SendGripperCommandRequest)
from kortex_driver.msg import GripperMode, Finger, Gripper
from kortex_driver.srv import GetMeasuredGripperMovement, GetMeasuredGripperMovementRequest


class GripperVelocityController:
    def __init__(self):
        rospy.init_node("left_arm_gripper_value_control", anonymous=False)

        self.input_topic = rospy.get_param("~input_topic", "/left_arm/gripper_value")
        self.rate_hz     = float(rospy.get_param("~rate",      40.0))
        self.kp          = float(rospy.get_param("~kp",        3.0))
        self.deadband    = float(rospy.get_param("~deadband",  0.005))
        self.max_speed   = float(rospy.get_param("~max_speed", 1.0))

        self._lock    = threading.Lock()
        self._desired = None   # target position 0.0-1.0

        # Wait for gripper command service
        svc = "/left_arm/base/send_gripper_command"
        rospy.loginfo("[gripper_vel] Waiting for %s ..." % svc)
        rospy.wait_for_service(svc, timeout=10.0)
        self._cmd_srv = rospy.ServiceProxy(svc, SendGripperCommand)

        # Wait for gripper feedback service
        fb_svc = "/left_arm/base/get_measured_gripper_movement"
        rospy.loginfo("[gripper_vel] Waiting for %s ..." % fb_svc)
        rospy.wait_for_service(fb_svc, timeout=10.0)
        self._fb_srv = rospy.ServiceProxy(fb_svc, GetMeasuredGripperMovement)

        rospy.Subscriber(self.input_topic, Float32, self._cb, queue_size=1)

        rospy.loginfo(
            "[gripper_vel] Ready. topic='%s'  rate=%.0fHz  kp=%.2f  deadband=%.3f"
            % (self.input_topic, self.rate_hz, self.kp, self.deadband)
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_position(self):
        """Return current gripper position 0.0-1.0, or None on failure."""
        try:
            resp = self._fb_srv(GetMeasuredGripperMovementRequest())
            if resp.output.finger:
                return float(resp.output.finger[0].value)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[gripper_vel] Feedback failed: %s" % e)
        return None

    def _send_speed(self, speed):
        """Send a speed command. Positive = close, negative = open."""
        req = SendGripperCommandRequest()
        req.input.mode = GripperMode.GRIPPER_SPEED
        f = Finger()
        f.finger_identifier = 1
        f.value = float(speed)   # signed: + close, - open
        req.input.gripper.finger.append(f)
        try:
            self._cmd_srv(req)
        except Exception as e:
            rospy.logerr_throttle(1.0, "[gripper_vel] Speed cmd failed: %s" % e)

    def _send_stop(self):
        self._send_speed(0.0)

    # ── Subscriber ────────────────────────────────────────────────────────────

    def _cb(self, msg):
        with self._lock:
            self._desired = max(0.0, min(1.0, float(msg.data)))

    # ── Control loop ──────────────────────────────────────────────────────────

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self._lock:
                desired = self._desired

            if desired is None:
                rate.sleep()
                continue

            actual = self._get_position()
            if actual is None:
                rate.sleep()
                continue

            error = desired - actual

            if abs(error) < self.deadband:
                self._send_stop()
            else:
                # Proportional speed — clamp to [-max_speed, max_speed]
                speed = self.kp * error
                speed = max(-self.max_speed, min(self.max_speed, speed))
                self._send_speed(speed)
                rospy.logdebug(
                    "[gripper_vel] actual=%.3f desired=%.3f err=%.3f spd=%.3f"
                    % (actual, desired, error, speed)
                )

            rate.sleep()

        self._send_stop()


if __name__ == "__main__":
    try:
        GripperVelocityController().spin()
    except rospy.ROSInterruptException:
        pass
