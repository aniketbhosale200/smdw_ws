import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class FruitDetector(Node):
    def __init__(self):
        super().__init__('fruit_detector')
        self.bridge = CvBridge()
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/gripper_rgbd_camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/gripper_rgbd_camera/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/gripper_rgbd_camera/color/camera_info', self.info_callback, 10)
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/fruit_pose', 10)
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Variables
        self.camera_matrix = None
        self.current_image = None
        self.current_depth = None

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        if self.current_image is not None and self.camera_matrix is not None:
            self.detect_fruit()

    def detect_fruit(self):
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        # Detect red fruit (for tomato)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Get the largest contour (assuming it's the fruit)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])  # Centroid x
                cy = int(M['m01'] / M['m00'])  # Centroid y
                depth = self.current_depth[cy, cx]
                if not np.isnan(depth):
                    # Calculate 3D position in camera frame
                    fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
                    cx_cam, cy_cam = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
                    X = (cx - cx_cam) * depth / fx
                    Y = (cy - cy_cam) * depth / fy
                    Z = depth

                    # Create pose in camera frame
                    pose = PoseStamped()
                    pose.header.frame_id = 'gripper_camera_optical'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = X
                    pose.pose.position.y = Y
                    pose.pose.position.z = Z
                    pose.pose.orientation.w = 1.0  # No rotation for simplicity

                    # Transform to base frame
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'base_link', 'gripper_camera_optical', rclpy.time.Time())
                        pose_base = do_transform_pose(pose, transform)
                        self.pose_pub.publish(pose_base)
                        self.get_logger().info('Tomato detected and pose published!')
                    except Exception as e:
                        self.get_logger().warn(f'TF transform failed: {e}')

def main():
    rclpy.init()
    node = FruitDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()