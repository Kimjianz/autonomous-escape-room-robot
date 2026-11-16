import rclpy
import subprocess
import re

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool


class DoorOpener(Node):

    def __init__(self):
        super().__init__('door_opener')

        self.key_removed = False
        self.door_reached = False
        self.door_opened = False
        self.open_attempted = False

        self.world_name = 'maze'
        self.door_name = 'red_door'

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 열쇠 획득 및 제거 상태 구독
        self.create_subscription(
            Bool,
            '/escape_room/key_removed',
            self.key_callback,
            qos
        )

        # 문 접근 완료 상태 구독
        self.create_subscription(
            Bool,
            '/escape_room/door_reached',
            self.door_callback,
            qos
        )

        # 문 개방 완료 상태 발행
        self.opened_pub = self.create_publisher(
            Bool,
            '/escape_room/door_opened',
            qos
        )

        self.opened_pub.publish(Bool(data=False))

        self.get_logger().info('Door Opener started')
        self.get_logger().info('Waiting for opening conditions')

    def key_callback(self, msg):

        self.key_removed = msg.data

        if msg.data:
            self.get_logger().info(
                'KEY_POSSESSION_CONFIRMED'
            )

        self.check_conditions()

    def door_callback(self, msg):

        self.door_reached = msg.data

        if msg.data:
            self.get_logger().info(
                'DOOR_REACH_CONFIRMED'
            )

        self.check_conditions()

    def check_conditions(self):

        if self.door_opened or self.open_attempted:
            return

        if self.key_removed and self.door_reached:

            self.open_attempted = True

            self.get_logger().info(
                'All opening conditions satisfied'
            )

            self.open_door()

    def open_door(self):

        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: "{self.door_name}" type: MODEL'
        ]

        try:

            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=10
            )

            if (
                result.returncode == 0
                and re.search(r'\bdata:\s*true\b', result.stdout)
            ):

                self.door_opened = True

                self.opened_pub.publish(
                    Bool(data=True)
                )

                self.get_logger().info('DOOR_OPENED')

            else:

                self.get_logger().error(
                    f'Failed to open door: '
                    f'{result.stdout} {result.stderr}'
                )

        except Exception as e:

            self.get_logger().error(str(e))


def main(args=None):

    rclpy.init(args=args)
    node = DoorOpener()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
