import rclpy
import math
import json
import statistics

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class DistanceTest(Node):

    def __init__(self):
        super().__init__('distance_test')

        self.key = None
        self.scan = None

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

        self.create_timer(1.0, self.check_distance)

        self.get_logger().info('Distance Test started')

    def scan_callback(self, msg):
        self.scan = msg

    def detection_callback(self, msg):
        detections = json.loads(msg.data)

        keys = [
            obj for obj in detections
            if obj['label'] == 'KEY'
        ]

        self.key = max(
            keys, key=lambda obj: obj['area']
        ) if keys else None

    def check_distance(self):

        if self.key is None or self.scan is None:
            self.get_logger().info('Waiting for KEY and LiDAR')
            return

        center_x = self.key['center_x']
        offset = center_x - 160.0

        if abs(offset) > 12:
            direction = 'LEFT' if offset < 0 else 'RIGHT'

            self.get_logger().info(
                f'KEY detected | x={center_x:.1f} | '
                f'Turn {direction}'
            )
            return

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
            self.get_logger().info('No valid distance')
            return

        distance = statistics.median(distances)

        self.get_logger().info(
            f'KEY centered | '
            f'Front distance={distance:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DistanceTest()

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
