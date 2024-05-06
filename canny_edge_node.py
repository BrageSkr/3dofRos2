import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdge(Node):
    """
    A node for blurring an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Gaussian blur node.
        """
        super().__init__('canny_edge')

        # Subscribe to an image topic
        self.subscription = self.create_subscription(
            Image,
            "blur_image",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the blurred image
        self.publisher = self.create_publisher(
            Image,
            'canny_image',
            10)

        # Initialize CVBridge
        self.bridge = CvBridge()

        self.declare_parameter('upper_treshold', 120)
        self.upper_treshold = self.get_parameter('upper_treshold').value

        self.declare_parameter('lower_treshold', 50)
        self.lower_treshold = self.get_parameter('lower_treshold').value


    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies Gaussian blur to the received image and publ_ishes the blurred image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        # Apply Gaussian blur
        try:
            edge_image = cv2.Canny(cv_image, self.lower_treshold, self.upper_treshold)
        except Exception as e:
            self.get_logger().error('Failed to apply edge: %s' % str(e))
            return

        # Convert back to ROS Image message
        try:
	        edge_msg = self.bridge.cv2_to_imgmsg(edge_image, "mono8")
        except Exception as e:
	        self.get_logger().error('Failed to convert image: %s' % str(e))
	        return

        # Publish the blurred image
        self.publisher.publish(edge_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    image_blur_node = CannyEdge()
    rclpy.spin(image_blur_node)
    image_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()