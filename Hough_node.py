import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tutorial_interfaces.msg import Pos
from cv_bridge import CvBridge

import numpy as np
import cv2

class Hough(Node):
    """
    A node for blurring an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Gaussian blur node.
        """
        super().__init__('hough')

        # Subscribe to an image topic
        self.subscription = self.create_subscription(
            Image,
            "canny_image",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the blurred image
        self.publisher = self.create_publisher(
            Image,
            'Hough_image',
            10)
        self.pos_publisher = self.create_publisher(
            Pos,
            'Hough_position',
            10)
        

        # Initialize CVBridge
        self.bridge = CvBridge()


    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies Gaussian blur to the received image and publishes the blurred image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

       

    # Convert the image to grayscale
        
        # Apply Hough transform
       # Apply Hough transform
        try:
            pos_msg = Pos()
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            detected_circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, 1, 10, param1=50, param2=30, minRadius=5, maxRadius=40)

            # Draw circles on the frame
            if detected_circles is not None:
                detected_circles = np.round(detected_circles[0, :]).astype("int")
                for (x, y, r) in detected_circles:
                    print(x, y, r)
                    pos_msg.x = int(x)
                    pos_msg.y = int(y)
                    pos_msg.r = int(r)
                    cv2.circle(cv_image, (x, y), r, (255, 255, 0), 3)

        except Exception as e:
            self.get_logger().error('Failed to apply Hough transform: %s' % str(e))
            return

        # Convert back to ROS Image message
        try:
            edge_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image end : %s' % str(e))
            return

        # Publish the modified image
        self.publisher.publish(edge_msg)
     
        self.pos_publisher.publish(pos_msg)



# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    image_blur_node = Hough()
    rclpy.spin(image_blur_node)
    image_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()