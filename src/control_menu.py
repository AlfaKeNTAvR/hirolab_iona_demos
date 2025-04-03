#!/usr/bin/env python
"""

Author(s):

TODO:

"""

# # Standart libraries:
import rospy
import cv2
from cv_bridge import (
    CvBridge,
    CvBridgeError,
)

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float64,
    String,
)
from sensor_msgs.msg import (Image)

# # Third party messages and services:
from kortex_driver.msg import (BaseCyclic_Feedback)


class ControlMenu:
    """
    
    """

    def __init__(
        self,
        node_name,
        image_topic,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__IMAGE_TOPIC = image_topic

        self.__BRIDGE = CvBridge()

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__cv_image = None
        self.__is_tracking = {
            'right_arm': False,
            'left_arm': False,
        }
        self.__is_full_mode = {
            'right_arm': False,
            'left_arm': False,
        }
        self.__preset_pose_mode = {
            'right_arm': False,
            'left_arm': False,
        }
        self.__preset_pose = {
            'right_arm': 'none',
            'left_arm': 'none',
        }
        self.__trajectory_fraction = {
            'right_arm': 0.0,
            'left_arm': 0.0,
        }
        self.__kinova_fault_state = {
            'right_arm': False,
            'left_arm': False,
        }

        self.__black_color = (0, 0, 0)
        self.__green_color = (0, 255, 0)
        self.__orange_color = (0, 140, 255)
        self.__red_color = (0, 0, 255)
        self.__purple_color = (255, 0, 255)

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {}

        self.__dependency_status['image_topic'] = False

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['image_topic'] = (
            rospy.Subscriber(
                f'{self.__IMAGE_TOPIC}',
                Image,
                self.__image_topic_callback,
            )
        )

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__control_menu_image = rospy.Publisher(
            f'{self.__NODE_NAME}/control_menu_image',
            Image,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.__IMAGE_TOPIC}',
            Image,
            self.__image_topic_callback,
        )

        rospy.Subscriber(
            '/right_arm/teleoperation/is_tracking',
            Bool,
            self.__right_arm_is_tracking_callback,
        )
        rospy.Subscriber(
            '/right_arm/teleoperation/position_only',
            Bool,
            self.__right_arm_position_only_callback,
        )
        rospy.Subscriber(
            '/right_arm/oculus_mapping/preset_pose_mode',
            Bool,
            self.__right_arm_preset_pose_mode_callback,
        )
        rospy.Subscriber(
            '/right_arm/oculus_mapping/preset_pose',
            String,
            self.__right_arm_preset_pose_callback,
        )
        rospy.Subscriber(
            '/right_arm/preset_poses/trajectory_fraction',
            Float64,
            self.__right_arm_trajectory_fraction_callback,
        )
        rospy.Subscriber(
            '/right_arm/base_feedback',
            BaseCyclic_Feedback,
            self.__right_base_feedback_callback,
        )

        rospy.Subscriber(
            '/left_arm/teleoperation/is_tracking',
            Bool,
            self.__left_arm_is_tracking_callback,
        )
        rospy.Subscriber(
            '/left_arm/teleoperation/position_only',
            Bool,
            self.__left_arm_position_only_callback,
        )
        rospy.Subscriber(
            '/left_arm/oculus_mapping/preset_pose_mode',
            Bool,
            self.__left_arm_preset_pose_mode_callback,
        )
        rospy.Subscriber(
            '/left_arm/oculus_mapping/preset_pose',
            String,
            self.__left_arm_preset_pose_callback,
        )
        rospy.Subscriber(
            '/left_arm/preset_poses/trajectory_fraction',
            Float64,
            self.__left_arm_trajectory_fraction_callback,
        )
        rospy.Subscriber(
            '/left_arm/base_feedback',
            BaseCyclic_Feedback,
            self.__left_base_feedback_callback,
        )

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors /<node_name>/is_initialized topic.
        
        """

        self.__dependency_status['image_topic'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __image_topic_callback(self, message):
        """
        
        """

        cv_image = None

        try:
            cv_image = self.__BRIDGE.imgmsg_to_cv2(
                message,
                'bgr8',
            )

        except CvBridgeError as e:
            rospy.logerr(
                (
                    f'{self.__NODE_NAME}:'
                    f' an error occured while converting from Image message to cv2. \n'
                    f'{e} \n'
                ),
            )
            return

        self.__cv_image = cv_image

        if not self.__is_initialized:
            self.__dependency_status['image_topic'] = True

    def __right_arm_is_tracking_callback(self, message):
        """
        
        """

        self.__is_tracking['right_arm'] = message.data

    def __right_arm_position_only_callback(self, message):
        """

        """

        self.__is_full_mode['right_arm'] = not message.data

    def __right_arm_preset_pose_mode_callback(self, message):
        """

        """

        self.__preset_pose_mode['right_arm'] = message.data

    def __right_arm_preset_pose_callback(self, message):
        """

        """

        self.__preset_pose['right_arm'] = message.data

    def __right_arm_trajectory_fraction_callback(self, message):
        """

        """

        self.__trajectory_fraction['right_arm'] = message.data

    def __right_base_feedback_callback(self, message: BaseCyclic_Feedback):
        """
        
        """

        if message.base.fault_bank_a != 0:
            self.__kinova_fault_state['right_arm'] = True

        else:
            self.__kinova_fault_state['right_arm'] = False

    def __left_arm_is_tracking_callback(self, message):
        """
        
        """

        self.__is_tracking['left_arm'] = message.data

    def __left_arm_position_only_callback(self, message):
        """

        """

        self.__is_full_mode['left_arm'] = not message.data

    def __left_arm_preset_pose_mode_callback(self, message):
        """

        """

        self.__preset_pose_mode['left_arm'] = message.data

    def __left_arm_preset_pose_callback(self, message):
        """

        """

        self.__preset_pose['left_arm'] = message.data

    def __left_arm_trajectory_fraction_callback(self, message):
        """

        """

        self.__trajectory_fraction['left_arm'] = message.data

    def __left_base_feedback_callback(self, message: BaseCyclic_Feedback):
        """
        
        """

        if message.base.fault_bank_a != 0:
            self.__kinova_fault_state['left_arm'] = True

        else:
            self.__kinova_fault_state['left_arm'] = False

    # # Timer callbacks:

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __make_padded_image(self, cv2_image):
        """

        """

        original_width = 640
        new_width = 1920
        padding_width = (new_width - original_width) // 2

        # Add padding to the sides
        padded_image = cv2.copyMakeBorder(
            cv2_image,
            0,
            0,
            padding_width,
            padding_width,
            cv2.BORDER_CONSTANT,
            value=[255, 255, 255],
        )

        # Right Arm text and colors
        text_start_x = 1300  # Start position for the text on the right
        text_start_y = 50  # Initial y-position
        line_spacing = 40  # Space between each line

        right_arm_text_lines = [
            (
                "Right Arm",
                self.__black_color,
            ),
            (
                f"",
                self.__black_color,
            ),
        ]

        if self.__kinova_fault_state['right_arm']:
            right_arm_text_lines.append(
                (
                    f"In FAULT state!",
                    self.__red_color,
                )
            ),
            right_arm_text_lines.append(
                (
                    f"Long press (B) to reset.",
                    self.__red_color,
                )
            ),

        elif self.__preset_pose_mode['right_arm']:
            right_arm_text_lines.append(
                (
                    f"Press (A) - next.",
                    self.__black_color,
                )
            )
            right_arm_text_lines.append(
                (
                    f"Press (B) - previous.",
                    self.__black_color,
                )
            )
            right_arm_text_lines.append(
                (
                    f"Long press (A) to confirm.",
                    self.__black_color,
                )
            )
            right_arm_text_lines.append((
                f"",
                self.__black_color,
            ))

            if self.__preset_pose['right_arm'] in [
                'front_xy',
                'front_xz',
                'top_xz',
                'top_yz',
                'side_yx',
                'side_yz',
            ]:
                image = cv2.imread(
                    f"/home/fetch/catkin_workspaces/iona_devel_ws/src/hirolab_iona_demos/images/{self.__preset_pose['right_arm']}_200x200.jpg"
                )

                image = cv2.resize(image, (200, 200))

                padded_image[250:450, 1300:1500] = image

            else:
                right_arm_text_lines.append(
                    (
                        f"{self.__preset_pose['right_arm']}",
                        self.__orange_color,
                    )
                )

        else:
            right_arm_text_lines.append(
                (
                    f"Tracking: {'ON.' if self.__is_tracking['right_arm'] else 'OFF.'}",
                    (
                        self.__orange_color if self.__is_tracking['right_arm']
                        else self.__black_color
                    ),
                )
            )
            right_arm_text_lines.append(
                (
                    f"Mode: {'with rotation.' if self.__is_full_mode['right_arm'] else 'position only.'}",
                    (
                        self.__orange_color if self.__is_full_mode['right_arm']
                        else self.__black_color
                    ),
                )
            )

        # Add each line for the right arm
        for i, (line, color) in enumerate(right_arm_text_lines):
            y_position = text_start_y + i * line_spacing
            cv2.putText(
                padded_image,  # Image to draw on
                line,  # Text to display
                (text_start_x, y_position),  # Position (x, y)
                cv2.FONT_HERSHEY_SIMPLEX,  # Font
                1.0,  # Font scale
                color,  # Font color
                2,  # Thickness
                cv2.LINE_AA,  # Line type
            )

        # Left Arm text and colors
        text_start_x = 200  # Start position for the text on the left
        text_start_y = 50  # Initial y-position

        left_arm_text_lines = [
            (
                "Left Arm",
                self.__black_color,
            ),
            (
                f"",
                self.__black_color,
            ),
        ]

        if self.__kinova_fault_state['left_arm']:
            left_arm_text_lines.append(
                (
                    f"In FAULT state!",
                    self.__red_color,
                )
            ),
            left_arm_text_lines.append(
                (
                    f"Long press (Y) to reset.",
                    self.__red_color,
                )
            ),

        elif self.__preset_pose_mode['left_arm']:
            left_arm_text_lines.append(
                (
                    f"Press (X) - next.",
                    self.__black_color,
                )
            )
            left_arm_text_lines.append(
                (
                    f"Press (Y) - previous.",
                    self.__black_color,
                )
            )
            left_arm_text_lines.append(
                (
                    f"Long press (X) to confirm.",
                    self.__black_color,
                )
            )
            left_arm_text_lines.append((
                f"",
                self.__black_color,
            ))

            if self.__preset_pose['left_arm'] in [
                'front_xy',
                'front_xz',
                'top_xz',
                'top_yz',
                'side_yx',
                'side_yz',
            ]:
                image = cv2.imread(
                    f"/home/fetch/catkin_workspaces/iona_devel_ws/src/hirolab_iona_demos/images/{self.__preset_pose['left_arm']}_200x200.jpg"
                )
                image = cv2.resize(image, (200, 200))

                if self.__preset_pose['left_arm'] in ['side_yx', 'side_yz']:
                    image = cv2.flip(image, 1)

                padded_image[250:450, 400:600] = image

            else:
                left_arm_text_lines.append(
                    (
                        f"{self.__preset_pose['left_arm']}",
                        self.__orange_color,
                    )
                )

        else:
            left_arm_text_lines.append(
                (
                    f"Tracking: {'ON.' if self.__is_tracking['left_arm'] else 'OFF.'}",
                    (
                        self.__orange_color if self.__is_tracking['left_arm']
                        else self.__black_color
                    ),
                )
            )
            left_arm_text_lines.append(
                (
                    f"Mode: {'with rotation.' if self.__is_full_mode['left_arm'] else 'position only.'}",
                    (
                        self.__orange_color if self.__is_full_mode['left_arm']
                        else self.__black_color
                    ),
                )
            )

        # Add each line for the left arm
        for i, (line, color) in enumerate(left_arm_text_lines):
            y_position = text_start_y + i * line_spacing
            cv2.putText(
                padded_image,  # Image to draw on
                line,  # Text to display
                (text_start_x, y_position),  # Position (x, y)
                cv2.FONT_HERSHEY_SIMPLEX,  # Font
                1.0,  # Font scale
                color,  # Font color
                2,  # Thickness
                cv2.LINE_AA,  # Line type
            )

        return padded_image

    def __publish_control_menu_image(self):
        """
        
        """

        cv2_image = self.__make_padded_image(self.__cv_image)

        ros_image_message = Image()
        ros_image_message = self.__BRIDGE.cv2_to_imgmsg(
            cv2_image,
            encoding='bgr8',
        )

        self.__control_menu_image.publish(ros_image_message)

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__publish_control_menu_image()

        # rospy.loginfo_throttle(1, f"{self.__is_tracking['right_arm']=}")

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'control_menu',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    image_topic = rospy.get_param(
        param_name=f'{rospy.get_name()}/image_topic',
        default='/active_neck/color/image_raw',
    )

    class_instance = ControlMenu(
        node_name=node_name,
        image_topic=image_topic,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
