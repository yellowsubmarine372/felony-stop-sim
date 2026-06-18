#!/usr/bin/env python3
"""
restricted_zone_detector.py

ROS2 노드: Turtlebot3 카메라 영상에서 경찰차 + 경광등 점멸을 감지하여
통제 구역을 판단하고, Nav2 obstacle layer에 가상 장애물을 publish한다.

구현 단계:
  1. /camera/image_raw 구독 -> CvBridge로 OpenCV 이미지 변환
  2. (계획) YOLOv8 fine-tuned 모델로 police_car bounding box 추론
  3. bounding box 내부 HSV 필터링으로 빨강/파랑 픽셀 비율 계산
  4. 최근 N 프레임에서의 색상 감지 비율이 임계값 이상이면 경광등 ON 판정
  5. 경광등 ON이면 PointCloud2로 가상 장애물 publish -> Nav2 costmap 반영

현재 상태: 스켈레톤. YOLO 추론 부분은 추후 fine-tuned 모델 로드 후 활성화.
"""

import collections
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import cv2
import struct


# 경광등 색상 HSV 범위 (제안서 명시)
RED_LOWER_1 = np.array([0, 100, 100])
RED_UPPER_1 = np.array([10, 255, 255])
RED_LOWER_2 = np.array([170, 100, 100])
RED_UPPER_2 = np.array([180, 255, 255])
BLUE_LOWER = np.array([100, 100, 100])
BLUE_UPPER = np.array([130, 255, 255])

# 다중 프레임 판단 파라미터
FRAME_WINDOW = 12              # 최근 N 프레임 (제안서: 10-15)
COLOR_RATIO_THRESHOLD = 0.30   # 윈도우 내 색상 감지 비율 임계값


class RestrictedZoneDetector(Node):
    def __init__(self):
        super().__init__('restricted_zone_detector')

        self.bridge = CvBridge()

        # 최근 프레임의 빨강/파랑 감지 여부 큐
        self.red_history = collections.deque(maxlen=FRAME_WINDOW)
        self.blue_history = collections.deque(maxlen=FRAME_WINDOW)

        # 구독
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # 가상 장애물 publish (Nav2 obstacle layer가 구독)
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/virtual_obstacle_points', 10
        )

        self.get_logger().info('Restricted zone detector started.')
        self.get_logger().info(f'Frame window: {FRAME_WINDOW}, threshold: {COLOR_RATIO_THRESHOLD}')

    def image_callback(self, msg: Image):
        """카메라 콜백. 경찰차 감지 + 경광등 판단 + 통제 구역 판정."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # 1) 경찰차 bounding box 검출 (YOLO 추론 자리)
        bbox = self.detect_police_car_bbox(cv_image)
        if bbox is None:
            # 경찰차 미감지 -> 히스토리에 0 push
            self.red_history.append(False)
            self.blue_history.append(False)
            return

        x1, y1, x2, y2 = bbox
        roi = cv_image[y1:y2, x1:x2]

        # 2) ROI 내부 HSV 필터링
        red_ratio, blue_ratio = self.compute_color_ratios(roi)

        # 단일 프레임에서 일정 픽셀 이상 검출되면 True
        SINGLE_FRAME_PIXEL_RATIO = 0.02
        self.red_history.append(red_ratio > SINGLE_FRAME_PIXEL_RATIO)
        self.blue_history.append(blue_ratio > SINGLE_FRAME_PIXEL_RATIO)

        # 3) 다중 프레임 판정
        red_detect_ratio = sum(self.red_history) / len(self.red_history)
        blue_detect_ratio = sum(self.blue_history) / len(self.blue_history)

        light_active = (
            red_detect_ratio > COLOR_RATIO_THRESHOLD
            and blue_detect_ratio > COLOR_RATIO_THRESHOLD
        )

        if light_active:
            self.get_logger().info(
                f'RESTRICTED ZONE DETECTED (red={red_detect_ratio:.2f}, '
                f'blue={blue_detect_ratio:.2f})'
            )
            # 4) 가상 장애물 publish
            self.publish_virtual_obstacle(bbox, cv_image.shape)

    def detect_police_car_bbox(self, image):
        """
        YOLOv8 fine-tuned 모델로 경찰차 bbox 반환.
        현재 스켈레톤: 모델 로드 전이므로 None 반환.
        모델 통합 후 구현 예정:
          - self.yolo = YOLO('weights/police_car_best.pt')  (init)
          - results = self.yolo(image, classes=[0], verbose=False)
          - bbox = results[0].boxes.xyxy[0].cpu().numpy().astype(int) if any
        """
        return None  # TODO: YOLO 추론 결과로 교체

    def compute_color_ratios(self, roi_bgr):
        """ROI(BGR) -> HSV 변환 -> 빨강/파랑 픽셀 비율."""
        if roi_bgr.size == 0:
            return 0.0, 0.0
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1) | \
                   cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
        blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

        total = roi_bgr.shape[0] * roi_bgr.shape[1]
        red_ratio = np.count_nonzero(red_mask) / total
        blue_ratio = np.count_nonzero(blue_mask) / total
        return red_ratio, blue_ratio

    def publish_virtual_obstacle(self, bbox, image_shape):
        """
        감지된 bbox 정보를 Turtlebot3 정면의 가상 장애물 좌표로 변환하여
        PointCloud2로 publish. Nav2 obstacle_layer가 이를 구독하면
        costmap에 동적 장애물로 반영됨.

        간소화 구현: bbox 중심 x 위치에 따라 정면 5m 거리에 점들 분포.
        실제 구현시 TF로 camera_link -> map 변환 필요.
        """
        h, w = image_shape[:2]
        x1, y1, x2, y2 = bbox
        bbox_cx = (x1 + x2) / 2

        # bbox 중심 x를 -1~1로 정규화 -> 정면 5m 거리, 좌우 ±1.5m
        lateral = ((bbox_cx / w) - 0.5) * 3.0
        forward = 5.0

        # 가상 장애물 점 클러스터
        points = []
        for dx in np.linspace(-0.3, 0.3, 5):
            for dy in np.linspace(-0.1, 0.1, 3):
                points.append((forward + dy, lateral + dx, 0.1))

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(struct.pack('fff', *p) for p in points)

        self.obstacle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RestrictedZoneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()