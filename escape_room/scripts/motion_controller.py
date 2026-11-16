import rclpy
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class MotionController(Node):

    def __init__(self):
        super().__init__('motion_controller')

        self.mode = 'STOP'
        self.command_timeout = 1.0

        self.nav_cmd = None
        self.approach_cmd = None

        self.nav_time = 0.0
        self.approach_time = 0.0

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Nav2 이동 명령 구독
        self.create_subscription(
            TwistStamped,
            '/cmd_vel_smoothed',
            self.navigation_callback,
            10
        )

        # 객체 접근 이동 명령 구독
        self.create_subscription(
            TwistStamped,
            '/cmd_vel_approach',
            self.approach_callback,
            10
        )

        # 이동 제어 모드 구독
        self.create_subscription(
            String,
            '/escape_room/control_mode',
            self.mode_callback,
            10
        )

        # 선택된 이동 명령 발행
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_selected',
            10
        )

        # 현재 제어 모드 발행
        self.mode_pub = self.create_publisher(
            String,
            '/escape_room/active_control_mode',
            qos
        )

        self.create_timer(0.1, self.control_loop)

        self.mode_pub.publish(String(data=self.mode))

        self.get_logger().info(
            'Motion Controller started'
        )

        self.get_logger().info('CONTROL_MODE: STOP')

    def navigation_callback(self, msg):

        if self.mode == 'NAVIGATION':
            self.nav_cmd = msg
            self.nav_time = self.get_clock().now().nanoseconds / 1e9

    def approach_callback(self, msg):

        if self.mode == 'APPROACH':
            self.approach_cmd = msg
            self.approach_time = self.get_clock().now().nanoseconds / 1e9

    def mode_callback(self, msg):

        new_mode = msg.data.strip().upper()

        if new_mode not in [
            'NAVIGATION',
            'APPROACH',
            'STOP'
        ]:
            self.get_logger().warning(
                f'Invalid mode: {new_mode}'
            )
            return

        if new_mode == self.mode:
            return

        # 제어권 전환 시 기존 명령 초기화
        self.nav_cmd = None
        self.approach_cmd = None

        self.nav_time = 0.0
        self.approach_time = 0.0

        self.mode = new_mode

        # 전환 순간 정지
        self.stop()

        self.mode_pub.publish(
            String(data=self.mode)
        )

        self.get_logger().info(
            f'CONTROL_MODE: {self.mode}'
        )

    def publish_command(self, command):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'
        msg.twist = command.twist

        self.cmd_pub.publish(msg)

    def stop(self):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        self.cmd_pub.publish(msg)

    def control_loop(self):

        now = self.get_clock().now().nanoseconds / 1e9

        # 자율 탐색 모드
        if self.mode == 'NAVIGATION':

            if (
                self.nav_cmd is not None
                and now - self.nav_time
                <= self.command_timeout
            ):
                self.publish_command(self.nav_cmd)

            else:
                self.stop()

        # 객체 접근 모드
        elif self.mode == 'APPROACH':

            if (
                self.approach_cmd is not None
                and now - self.approach_time
                <= self.command_timeout
            ):
                self.publish_command(
                    self.approach_cmd
                )

            else:
                self.stop()

        # 정지 모드
        else:
            self.stop()


def main(args=None):

    rclpy.init(args=args)
    node = MotionController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
