import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class IMUHeadingPublisher(Node):
    def __init__(self):
        super().__init__('imu_to_heading')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Float64, 'yaw_heading', 10)

    def imu_callback(self, msg):
        # 쿼터니언 데이터 추출
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Yaw 계산 (z 축 주위 회전 각도)
        # Yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 라디안에서 도(degree)로 변환
        yaw_degrees = math.degrees(yaw)

        # Normalize the angle to be within the range [0, 360)
        if yaw_degrees < 0:
            yaw_degrees += 360

        heading_msg = Float64()
        heading_msg.data = yaw_degrees
        self.publisher.publish(heading_msg)
        # self.get_logger().info('Published yaw heading: {:.2f} degrees'.format(yaw_degrees))

def main(args=None):
    rclpy.init(args=args)
    node = IMUHeadingPublisher()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
