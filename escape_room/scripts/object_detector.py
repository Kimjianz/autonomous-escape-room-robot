import cv2
import numpy as np
import rclpy
import json

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            Image,
            '/escape_room/detection_image',
            10
        )

        # 인식 결과 데이터 발행
        self.detection_publisher = self.create_publisher(
            String,
            '/escape_room/detections',
            10
        )

        self.get_logger().info('Object Detector started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 노란색 열쇠
        yellow_mask = cv2.inRange(
            hsv, np.array([20, 100, 50]),
            np.array([35, 255, 255])
        )

        # 빨간색 문
        red_mask1 = cv2.inRange(
            hsv, np.array([0, 100, 50]),
            np.array([10, 255, 255])
        )

        red_mask2 = cv2.inRange(
            hsv, np.array([170, 100, 50]),
            np.array([179, 255, 255])
        )

        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # 초록색 탈출구
        green_mask = cv2.inRange(
            hsv, np.array([40, 100, 50]),
            np.array([85, 255, 255])
        )

        detections = []

        detections.extend(
            self.detect(frame, yellow_mask, 'KEY', (0, 255, 255))
        )

        detections.extend(
            self.detect(frame, red_mask, 'DOOR', (0, 0, 255))
        )

        detections.extend(
            self.detect(frame, green_mask, 'EXIT', (0, 255, 0))
        )

        # 인식 결과 영상 발행
        output = self.bridge.cv2_to_imgmsg(
            frame, encoding='bgr8'
        )

        output.header = msg.header
        self.publisher.publish(output)

        # 인식 결과 데이터 발행
        detection_msg = String()
        detection_msg.data = json.dumps(detections)

        self.detection_publisher.publish(detection_msg)

    def detect(self, frame, mask, label, color):

        detections = []

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            cv2.rectangle(
                frame, (x, y), (x + w, y + h),
                color, 2
            )

            cv2.putText(
                frame, label,
                (x, max(15, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, color, 2
            )

            # 물체 정보 저장
            detections.append({
                'label': label,
                'center_x': x + w / 2,
                'center_y': y + h / 2,
                'width': w,
                'height': h,
                'area': area
            })

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

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
