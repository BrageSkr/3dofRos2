import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Pos
from std_msgs.msg import Float32MultiArray
import numpy as np

# Erstatt disse med dine faktiske kalibreringsverdier
alpha = 1
beta = 1
x0 = 1
y0 = 0
theta = 1

# Kalibreringsmatrisen K
K = np.array([[alpha, -alpha * np.tan(theta), x0],
              [0, beta / np.sin(theta), y0],
              [0, 0, 1]])

# Beregner den inverse av K
K_inv = np.linalg.inv(K)

class CirclePositionNode(Node):
    def __init__(self):
        super().__init__('circle_position_publisher')
        self.subscription = self.create_subscription(
            Pos,
            "Hough_position",
            self.callback,
            10)
    
        # Create a publisher for 3D position
        self.publisher_ = self.create_publisher(Float32MultiArray, 'circle_pos', 10)

    def callback(self, msg):
        
        x_pixel = msg.x
        y_pixel = msg.y
        
        # Publish 3D position
        self.publish_3D_position(x_pixel, y_pixel, Z=0.3)  # Example depth value

    def calculate_3D_position(self, x_pixel, y_pixel, Z):
        # Konverter pikselkoordinater til homogene koordinater
        pixel_homogeneous = np.array([x_pixel, y_pixel, 1])
        # Konverter pikselkoordinater til normaliserte bildekoordinater
        normalized_image_coordinates = K_inv.dot(pixel_homogeneous)
        # Skaler med dybdeinformasjonen, finne 3D-posisjonen
        X = normalized_image_coordinates[0] * Z
        Y = normalized_image_coordinates[1] * Z
        ball_position_3D = np.array([X, Y, Z])
        return ball_position_3D

    def publish_3D_position(self, x_pixel, y_pixel, Z):
        ball_position_3D = self.calculate_3D_position(x_pixel, y_pixel, Z)
        # Publish the 3D position
        msg = Float32MultiArray()
        msg.data = ball_position_3D.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    circle_position_node = CirclePositionNode()
    rclpy.spin(circle_position_node)
    circle_position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
