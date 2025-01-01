import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMultiplier(Node):

    def __init__(self):
        super().__init__('cmd_vel_multiplier')
        self.subscription = self.create_subscription(
            Twist,
            'rm_cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_scaled', 10)
        self.multiplier = 6.0  # 设置倍数，默认为2

    def cmd_vel_callback(self, msg):
        scaled_twist = Twist()
        scaled_twist.linear = msg.linear
        scaled_twist.angular = msg.angular
        scaled_twist.angular.z *= self.multiplier  # 将角速度乘以倍数
        self.publisher.publish(scaled_twist)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_multiplier = CmdVelMultiplier()
    rclpy.spin(cmd_vel_multiplier)
    cmd_vel_multiplier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
