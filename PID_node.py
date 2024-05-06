import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'circle_pos',
            self.position_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pid_output', 10)

        # Separate PID parameters for X and Y
        self.kp = {'x': 1.0, 'y': 1.2}
        self.ki = {'x': 0.1, 'y': 0.1}
        self.kd = {'x': 0.05, 'y': 0.06}

        # Separate PID state for X and Y
        self.integral = {'x': 0, 'y': 0}
        self.prev_error = {'x': 0, 'y': 0}

    def position_callback(self, msg):
        # Assuming the message contains X, Y, Z positions
        current_position = msg.data  

        # Define desired positions for X and Y
        desired_position = {'x': 100, 'y': 100}  

        # Calculate errors for X and Y
        error = {
            'x': desired_position['x'] - current_position[0],
            'y': desired_position['y'] - current_position[1]
        }

        # Update PID for each direction
        output = {
            'x': self.update_pid('x', error['x']),
            'y': self.update_pid('y', error['y'])
        }

        # Publish PID outputs
        self.publish_pid_output(output)

    def update_pid(self, direction, error):
        # Update integral
        self.integral[direction] = error

        # Calculate derivative
        derivative = error - self.prev_error[direction]

        # PID output for the direction
        pid_output = (self.kp[direction] * error +
                      self.ki[direction] * self.integral[direction] +
                      self.kd[direction] * derivative)

        # Update previous error for the direction
        self.prev_error[direction] = error

        return pid_output

    def publish_pid_output(self, output):
        msg = Float32MultiArray()
        msg.data = [output['x'], output['y']]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
