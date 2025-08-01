#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import subprocess
import re


def get_wifi_strength():
    """
    Uses iwconfig to get the Wi-Fi signal strength in dBm.
    Returns signal level in dBm as a float (negative value) or None if not found.
    """
    try:
        iwconfig_output = subprocess.check_output(
            ["iwconfig"], stderr=subprocess.DEVNULL
        ).decode()
        # Look for 'Signal level=-XX dBm'
        match = re.search(r"Signal level=(-?\d+)\s*dBm", iwconfig_output)
        if match:
            return float(match.group(1))
    except Exception as e:
        rospy.logwarn("Could not get Wi-Fi strength: %s", e)
    return None


def wifi_publisher():
    rospy.init_node('wifi_strength_publisher')
    pub = rospy.Publisher('/connection', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        strength = get_wifi_strength()
        if strength is not None:
            # rospy.loginfo("Wi-Fi Signal Strength: %s dBm", strength)
            pub.publish(strength)
        else:
            rospy.logwarn("Wi-Fi signal strength not available")
        rate.sleep()


if __name__ == '__main__':
    try:
        wifi_publisher()
    except rospy.ROSInterruptException:
        pass
