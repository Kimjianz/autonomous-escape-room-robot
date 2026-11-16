import rclpy
import subprocess
import re

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool


class KeyRemover(Node):

    def __init__(self):
        super().__init__('key_remover')

        self.key_removed = False

        self.world_name = 'maze'
        self.key_name = 'red_key'

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 열쇠 획득 상태 구독
        self.create_subscription(
            Bool,
            '/escape_room/key_acquired',
            self.key_callback,
            qos
        )

        # 열쇠 제거 완료 상태 발행
        self.removed_pub = self.create_publisher(
            Bool,
            '/escape_room/key_removed',
            qos
        )

        self.removed_pub.publish(Bool(data=False))

        self.get_logger().info('Key Remover started')
        self.get_logger().info('Waiting for key acquisition')

    def key_callback(self, msg):

        if not msg.data or self.key_removed:
            return

        self.get_logger().info('Key acquisition confirmed')

        self.remove_key()

    def remove_key(self):

        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: "{self.key_name}" type: MODEL'
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
                self.key_removed = True

                self.removed_pub.publish(Bool(data=True))

                self.get_logger().info('KEY_REMOVED')

            else:
                self.get_logger().error(
                    f'Failed to remove key: '
                    f'{result.stdout} {result.stderr}'
                )

        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = KeyRemover()

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
