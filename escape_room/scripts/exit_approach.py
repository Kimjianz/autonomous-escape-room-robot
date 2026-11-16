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
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


class ExitApproach(Node):

    def __init__(self):
        super().__init__('exit_approach')

        # 초기 설정값
        self.target_distance = 0.5
        self.image_center = 160.0
        self.center_tolerance = 6.0
        self.required_count = 5
        self.passage_distance = 1.0

        self.door_opened = False
        self.door_passed = False
        self.escape_success = False

        self.exit = None
        self.scan = None
        self.position = None
        self.start_position = None

        self.detection_time = 0.0
        self.scan_time = 0.0
        self.odom_time = 0.0

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
            '/escape_room/door_opened',
            self.door_callback,
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

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )

        self.success_pub = self.create_publisher(
            Bool,
            '/escape_room/escape_success',
            qos
        )

        self.success_pub.publish(Bool(data=False))

        self.create_timer(0.2, self.control_loop)

        self.get_logger().info('Exit Approach started')

    def door_callback(self, msg):

        self.door_opened = msg.data

        if msg.data:
            self.get_logger().info(
                'DOOR_OPEN_CONFIRMED'
            )

    def scan_callback(self, msg):

        self.scan = msg
        self.scan_time = time.monotonic()

    def odom_callback(self, msg):

        position = msg.pose.pose.position

        self.position = (position.x, position.y)
        self.odom_time = time.monotonic()

    def detection_callback(self, msg):

        detections = json.loads(msg.data)

        exits = [
            obj for obj in detections
            if obj['label'] == 'EXIT'
        ]

        self.exit = max(
            exits, key=lambda obj: obj['area']
        ) if exits else None

        self.detection_time = time.monotonic()
        self.frame_count += 1

        if self.exit is None:
            self.confirm_count = 0

    def front_distance(self, width=3, nearest=False):

        distances = []
        clear = False

        for i, distance in enumerate(self.scan.ranges):

            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            angle = math.atan2(
                math.sin(angle), math.cos(angle)
            )

            if abs(angle) <= math.radians(width):

                if distance == math.inf:
                    clear = True

                elif (
                    math.isfinite(distance)
                    and self.scan.range_min
                    <= distance
                    <= self.scan.range_max
                ):
                    distances.append(distance)

        if distances:
            if nearest:
                return min(distances)

            return statistics.median(distances)

        if nearest and clear:
            return math.inf

        return None

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

        # 다른 이동 제어 노드가 종료될 때까지 대기
        if self.count_publishers('/cmd_vel') > 1:

            self.confirm_count = 0
            self.set_state('WAITING_FOR_CONTROL')
            return

        if self.escape_success:

            self.stop()
            return

        if not self.door_opened:

            self.stop()
            self.set_state('WAITING_FOR_DOOR')
            return

        now = time.monotonic()

        # 센서 데이터 유효성 확인
        if (
            self.scan is None
            or self.position is None
            or now - self.scan_time > 3.0
            or now - self.odom_time > 3.0
            or now - self.detection_time > 3.0
        ):

            self.stop()
            self.confirm_count = 0
            self.set_state('WAITING_FOR_SENSORS')
            return

        # 1. 열린 문 통과
        if not self.door_passed:

            if self.start_position is None:
                self.start_position = self.position

            moved = math.hypot(
                self.position[0] - self.start_position[0],
                self.position[1] - self.start_position[1]
            )

            if moved >= self.passage_distance:

                self.stop()
                self.door_passed = True
                self.set_state('DOOR_PASSED')
                return

            distance = self.front_distance(
                width=20, nearest=True
            )

            if distance is None or distance < 0.3:

                self.stop()
                self.set_state('PASSAGE_BLOCKED')
                return

            self.move(0.08, 0.0)
            self.set_state('PASS_THROUGH_DOOR')
            return

        # 2. 탈출구 탐색
        if self.exit is None:

            self.confirm_count = 0

            sim_now = (
                self.get_clock().now().nanoseconds / 1e9
            )

            if self.search_start is None:
                self.search_start = sim_now

            if sim_now - self.search_start > 35.0:

                self.stop()
                self.set_state('EXIT_NOT_FOUND')
                return

            self.move(0.0, 0.2)
            self.set_state('SEARCHING_FOR_EXIT')
            return

        self.search_start = None

        # 3. 탈출구 방향 정렬
        offset = (
            self.exit['center_x'] - self.image_center
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

        # 4. 탈출구까지 거리 측정
        distance = self.front_distance()

        if distance is None:

            self.stop()
            self.confirm_count = 0
            self.set_state('INVALID_DISTANCE')
            return

        # 5. 탈출구 자율 접근
        if distance > self.target_distance:

            self.confirm_count = 0

            obstacle = self.front_distance(
                width=20, nearest=True
            )

            if obstacle is None or obstacle < 0.3:

                self.stop()
                self.set_state('EXIT_PATH_BLOCKED')
                return

            speed = min(
                0.08,
                0.3 * (distance - self.target_distance)
                + 0.02
            )

            self.move(speed, 0.0)
            self.set_state('APPROACH_EXIT')

        else:

            self.stop()
            self.set_state('CONFIRMING_EXIT')

            if self.frame_count != self.last_checked_frame:

                self.last_checked_frame = self.frame_count
                self.confirm_count += 1

                self.get_logger().info(
                    f'Distance={distance:.3f} m | '
                    f'Confirm={self.confirm_count}/5'
                )

            # 6. 탈출 성공
            if self.confirm_count >= self.required_count:

                self.escape_success = True

                self.stop()

                self.success_pub.publish(
                    Bool(data=True)
                )

                self.set_state('ESCAPE_SUCCESS')


def main(args=None):

    rclpy.init(args=args)
    node = ExitApproach()

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
