#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import numpy as np
from image_geometry import PinholeCameraModel

class ColoredPointCloudNode:
    def __init__(self):
        rospy.init_node('colored_pointcloud_node', log_level=rospy.INFO)

        # Parameters
        self.max_distance = rospy.get_param("~max_distance", 0.30)  # meters
        self.min_distance = rospy.get_param("~min_distance", 0.05)  # meters
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.got_cam_info = False
        self.downsample_factor = rospy.get_param("~downsample_factor", 1)
        self.min_points_threshold = 10

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw", Image,
                         self.depth_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback,
                         queue_size=1)

        self.pub = rospy.Publisher("/camera/colored_near_points", PointCloud2,
                                   queue_size=1)

        # Synchronization variables
        self.last_rgb = None
        self.last_depth = None
        self.last_depth_time = None
        self.last_rgb_time = None
        self.camera_matrix = None
        self.frame_id = None

        # Counters
        self.rgb_count = 0
        self.depth_count = 0
        self.processing_count = 0

        # Processing timer
        rospy.Timer(rospy.Duration(0.05), self.process_data)

    def info_callback(self, msg):
        if not self.got_cam_info:
            self.cam_model.fromCameraInfo(msg)
            self.got_cam_info = True
            self.camera_matrix = np.array([[self.cam_model.fx(), 0, self.cam_model.cx()],
                                           [0, self.cam_model.fy(), self.cam_model.cy()],
                                           [0, 0, 1]])
            rospy.loginfo("Got camera intrinsics. fx:%.1f fy:%.1f cx:%.1f cy:%.1f" %
                          (self.cam_model.fx(), self.cam_model.fy(),
                           self.cam_model.cx(), self.cam_model.cy()))

    def rgb_callback(self, msg):
        self.rgb_count += 1
        try:
            self.last_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_rgb_time = msg.header.stamp
            self.frame_id = msg.header.frame_id

            if self.downsample_factor > 1:
                self.last_rgb = cv2.resize(
                    self.last_rgb,
                    (self.last_rgb.shape[1] // self.downsample_factor,
                     self.last_rgb.shape[0] // self.downsample_factor),
                    interpolation=cv2.INTER_NEAREST)

        except Exception as e:
            rospy.logerr("RGB Error: %s", str(e))

    def depth_callback(self, msg):
        self.depth_count += 1
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth_time = msg.header.stamp

            if self.downsample_factor > 1:
                self.last_depth = cv2.resize(
                    self.last_depth,
                    (self.last_depth.shape[1] // self.downsample_factor,
                     self.last_depth.shape[0] // self.downsample_factor),
                    interpolation=cv2.INTER_NEAREST)

        except Exception as e:
            rospy.logerr("Depth Error: %s", str(e))

    def process_data(self, event):
        self.processing_count += 1

        if not self.got_cam_info or self.last_rgb is None or self.last_depth is None:
            return

        time_diff = abs((self.last_rgb_time - self.last_depth_time).to_sec())
        if time_diff > 0.1:
            rospy.logwarn_throttle(1.0, f"Images not synchronized! Δt: {time_diff:.3f}s")
            return

        if self.last_rgb.shape[:2] != self.last_depth.shape:
            rospy.logwarn_throttle(1.0, f"Shape mismatch RGB: {self.last_rgb.shape} Depth: {self.last_depth.shape}")
            return

        # Convert to meters
        depth_meters = self.last_depth.astype(np.float32) * 0.001

        # Apply median filter to reduce speckle noise near zero
        depth_meters = cv2.medianBlur(depth_meters, 5)

        # Valid mask with min_distance
        valid_mask = (depth_meters >= self.min_distance) & (depth_meters <= self.max_distance)

        v_coords, u_coords = np.where(valid_mask)
        num_valid_points = len(v_coords)

        if num_valid_points < self.min_points_threshold:
            return

        z_values = depth_meters[v_coords, u_coords]
        uv_homogeneous = np.column_stack([u_coords * self.downsample_factor,
                                          v_coords * self.downsample_factor,
                                          np.ones_like(u_coords)])
        xy_homogeneous = np.linalg.inv(self.camera_matrix) @ uv_homogeneous.T
        x_values = xy_homogeneous[0] * z_values
        y_values = xy_homogeneous[1] * z_values

        colors = self.last_rgb[v_coords, u_coords]
        colors = np.clip(colors.astype(np.uint8), 0, 255)
        rgb_packed = ((colors[:, 0].astype(np.uint32)) |
                      (colors[:, 1].astype(np.uint32) << 8) |
                      (colors[:, 2].astype(np.uint32) << 16))

        points = np.zeros(num_valid_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])
        points['x'] = x_values
        points['y'] = y_values
        points['z'] = z_values
        points['rgb'] = rgb_packed

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud_msg)

if __name__ == '__main__':
    try:
        ColoredPointCloudNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

