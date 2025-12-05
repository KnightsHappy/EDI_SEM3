import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.get_logger().info('Image Viewer Node has started.')

        # Create a subscriber to the raw image topic
        self.subscription_raw = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.raw_image_callback,
            10)

        # Create a subscriber to the depth image topic
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_image_callback,
            10)

        self.bridge = CvBridge()

    def raw_image_callback(self, msg):
        """Callback function for the raw image subscriber."""
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Display the image
        cv2.imshow("Raw Camera View", cv_image)
        cv2.waitKey(1)

    def depth_image_callback(self, msg):
        """Callback function for the depth image subscriber."""
        try:
            # The depth image is a single-channel float32 image
            # Convert it to a format that OpenCV can display (e.g., 8-bit grayscale)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Normalize the depth image for better visualization
            # We will cover this in more detail later. For now, it just makes it look better.
            cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        # Display the image
        cv2.imshow("Depth Camera View", cv_image_normalized)
        cv2.waitKey(1)

def main(rclpy=rclpy):
    rclpy.init()
    image_viewer_node = ImageViewer()
    rclpy.spin(image_viewer_node)
    # Destroy the node explicitly
    image_viewer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()