import rclpy
import math
import json
import statistics
import time

from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    DurabilityPolicy
)
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String, Bool


class KeyApproach(Node):

    def __init__(self):
        super().__init__('key_approach')

        # 초기 설정값
        self.target_distance = 0.5
        self.image_center = 160.0
        self.center_tolerance = 6.0
        self.required_count = 5

        self.key = None
        self.scan = None
        self.key_acquired = False
        self.confirm_count = 0

        self.detection_time = 0.0
        self.scan_time = 0.0
        self.frame_count = 0
        self.last_checked_frame = -1
        self.state = None

        # 이동 명령 메시지 타입
        self.declare_parameter('stamped_cmd_vel', True)
        self.stamped = self.get_parameter(
            'stamped_cmd_vel'
        ).value

        msg_type = TwistStamped if self.stamped else Twist

        self.cmd_pub = self.create_publisher(
            msg_type, '/cmd_vel', 10
        )

        # 열쇠 획득 상태 발행
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.key_pub = self.create_publisher(
            Bool, '/escape_room/key_acquired', qos
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            '/escape_room/detections',
            self.detection_callback,
            10
        )

        self.create_timer(0.2, self.control_loop)

        self.key_pub.publish(Bool(data=False))

        self.get_logger().info('Key Approach started')

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def detection_callback(self, msg):
        detections = json.loads(msg.data)

        keys = [
            obj for obj in detections
            if obj['label'] == 'KEY'
        ]

        self.key = max(
            keys, key=lambda obj: obj['area']
        ) if keys else None

        self.detection_time = time.monotonic()
        self.frame_count += 1

        if self.key is None:
            self.confirm_count = 0

    def front_distance(self):
        distances = []

        for i, distance in enumerate(self.scan.ranges):

            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            angle = math.atan2(
                math.sin(angle), math.cos(angle)
            )

            if abs(angle) <= math.radians(3):
                if (
                    math.isfinite(distance)
                    and self.scan.range_min
                    <= distance
                    <= self.scan.range_max
                ):
                    distances.append(distance)

        if not distances:
            return None

        return statistics.median(distances)

    def move(self, linear=0.0, angular=0.0):

        velocity = Twist()
        velocity.linear.x = float(linear)
        velocity.angular.z = float(angular)

        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist = velocity
        else:
            msg = velocity

        self.cmd_pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)

    def set_state(self, state):
        if self.state != state:
            self.state = state
            self.get_logger().info(state)

    def control_loop(self):

        # 이미 획득했으면 정지 상태 유지
        if self.key_acquired:
            self.stop()
            return

        now = time.monotonic()

        # 센서 데이터가 없거나 오래된 경우 정지
        if (
            self.key is None
            or self.scan is None
            or now - self.detection_time > 2.0
            or now - self.scan_time > 2.0
        ):
            self.stop()
            self.confirm_count = 0
            self.set_state('WAITING_FOR_KEY')
            return

        offset = self.key['center_x'] - self.image_center

        # 1. 열쇠 방향으로 회전
        if abs(offset) > self.center_tolerance:

            self.confirm_count = 0

            angular = max(
                -0.25,
                min(0.25, -0.008 * offset)
            )

            self.move(0.0, angular)

            self.set_state(
                'ALIGN_LEFT' if offset < 0
                else 'ALIGN_RIGHT'
            )
            return

        # 2. 정면 거리 확인
        distance = self.front_distance()

        if distance is None:
            self.stop()
            self.confirm_count = 0
            self.set_state('INVALID_DISTANCE')
            return

        # 3. 열쇠 방향으로 전진
        if distance > self.target_distance:

            self.confirm_count = 0

            speed = min(
                0.08,
                0.3 * (distance - self.target_distance)
                + 0.02
            )

            self.move(speed, 0.0)
            self.set_state('APPROACH')

        # 4. 획득 조건 확인
        else:

            self.stop()
            self.set_state('CONFIRMING_KEY')

            # 같은 인식 데이터를 중복 계산하지 않음
            if self.frame_count != self.last_checked_frame:

                self.last_checked_frame = self.frame_count
                self.confirm_count += 1

                self.get_logger().info(
                    f'Distance={distance:.3f} m | '
                    f'Confirm={self.confirm_count}/5'
                )

            # 5. 열쇠 획득 및 기억
            if self.confirm_count >= self.required_count:

                self.key_acquired = True

                self.stop()
                self.key_pub.publish(Bool(data=True))

                self.set_state('KEY_ACQUIRED')


def main(args=None):
    rclpy.init(args=args)
    node = KeyApproach()

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
