import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64
import math

class MagnetometerHeadingPublisher(Node):
    def __init__(self):
        super().__init__('magnetometer_to_heading')

        self.axis_orientation = 'Y_NORTH'  # 여기서 원하는 방향으로 설정하세요.
        self.subscription = self.create_subscription(
            MagneticField,
            'magnetometer',
            self.magnetometer_callback,
            10)
        self.publisher = self.create_publisher(Float64, 'heading', 10)

    def magnetometer_callback(self, msg):
        Bx = msg.magnetic_field.x
        By = msg.magnetic_field.y
        
        # 센서 배치에 따라 각도 계산
        if self.axis_orientation == 'X_NORTH':
            theta_radians = math.atan2(By, Bx)
        elif self.axis_orientation == 'Y_NORTH':
            theta_radians = math.atan2(-Bx, By)  # +Y가 북쪽일 때의 계산

        theta_degrees = math.degrees(theta_radians)
        
        # Normalize the angle to be within the range [0, 360)
        if theta_degrees < 0:
            theta_degrees += 360

        heading_msg = Float64()
        heading_msg.data = theta_degrees
        self.publisher.publish(heading_msg)
        self.get_logger().info('Published heading: {:.2f} degrees'.format(theta_degrees))

def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerHeadingPublisher()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

