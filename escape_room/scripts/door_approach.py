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
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Bool


class DoorApproach(Node):

    def __init__(self):
        super().__init__('door_approach')

        self.target_distance = 0.5
        self.image_center = 160.0
        self.center_tolerance = 6.0
        self.required_count = 5

        self.key_acquired = False
        self.key_removed = False
        self.door_reached = False

        self.door = None
        self.scan = None

        self.detection_time = 0.0
        self.scan_time = 0.0

        self.frame_count = 0
        self.last_checked_frame = -1
        self.confirm_count = 0

        self.search_start = None
        self.state = None

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            Bool,
            '/escape_room/key_acquired',
            self.key_callback,
            qos
        )

        self.create_subscription(
            Bool,
            '/escape_room/key_removed',
            self.removed_callback,
            qos
        )

        self.create_subscription(
            String,
            '/escape_room/detections',
            self.detection_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )

        self.reached_pub = self.create_publisher(
            Bool,
            '/escape_room/door_reached',
            qos
        )

        self.reached_pub.publish(Bool(data=False))

        self.create_timer(0.2, self.control_loop)

        self.get_logger().info('Door Approach started')

    def key_callback(self, msg):
        self.key_acquired = msg.data

        if msg.data:
            self.get_logger().info(
                'KEY_POSSESSION_CONFIRMED'
            )

    def removed_callback(self, msg):
        self.key_removed = msg.data

        if msg.data:
            self.get_logger().info(
                'KEY_REMOVAL_CONFIRMED'
            )

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def detection_callback(self, msg):

        detections = json.loads(msg.data)

        doors = [
            obj for obj in detections
            if obj['label'] == 'DOOR'
        ]

        self.door = max(
            doors, key=lambda obj: obj['area']
        ) if doors else None

        self.detection_time = time.monotonic()
        self.frame_count += 1

        if self.door is None:
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

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)

    def set_state(self, state):

        if self.state != state:
            self.state = state
            self.get_logger().info(state)

    def control_loop(self):

        # 다른 이동 제어 노드가 있으면 제어권 대기
        if self.count_publishers('/cmd_vel') > 1:
            self.confirm_count = 0
            self.set_state('WAITING_FOR_CONTROL')
            return

        # 열쇠 획득 및 제거 여부 확인
        if not self.key_acquired or not self.key_removed:

            self.stop()
            self.set_state('WAITING_FOR_KEY')
            return

        if self.door_reached:
            self.stop()
            return

        now = time.monotonic()

        # 센서 데이터 유효성 확인
        if (
            self.scan is None
            or now - self.scan_time > 3.0
            or now - self.detection_time > 3.0
        ):
            self.stop()
            self.confirm_count = 0
            self.set_state('WAITING_FOR_SENSORS')
            return

        # 1. 문 찾기
        if self.door is None:

            self.confirm_count = 0

            sim_now = (
                self.get_clock().now().nanoseconds / 1e9
            )

            if self.search_start is None:
                self.search_start = sim_now

            if sim_now - self.search_start > 35.0:

                self.stop()
                self.set_state('DOOR_NOT_FOUND')
                return

            self.move(0.0, 0.2)
            self.set_state('SEARCHING_FOR_DOOR')
            return

        self.search_start = None

        # 2. 문 방향으로 회전
        offset = (
            self.door['center_x'] - self.image_center
        )

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

        # 3. 문까지의 거리 확인
        distance = self.front_distance()

        if distance is None:

            self.stop()
            self.confirm_count = 0
            self.set_state('INVALID_DISTANCE')
            return

        # 4. 문 방향으로 전진
        if distance > self.target_distance:

            self.confirm_count = 0

            speed = min(
                0.08,
                0.3 * (distance - self.target_distance)
                + 0.02
            )

            self.move(speed, 0.0)
            self.set_state('APPROACH_DOOR')

        else:

            self.stop()
            self.set_state('CONFIRMING_DOOR')

            if self.frame_count != self.last_checked_frame:

                self.last_checked_frame = self.frame_count
                self.confirm_count += 1

                self.get_logger().info(
                    f'Distance={distance:.3f} m | '
                    f'Confirm={self.confirm_count}/5'
                )

            # 5. 문 접근 완료
            if self.confirm_count >= self.required_count:

                self.door_reached = True

                self.stop()

                self.reached_pub.publish(
                    Bool(data=True)
                )

                self.set_state('DOOR_REACHED')


def main(args=None):

    rclpy.init(args=args)
    node = DoorApproach()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if (
            rclpy.ok()
            and node.count_publishers('/cmd_vel') == 1
        ):
            node.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
