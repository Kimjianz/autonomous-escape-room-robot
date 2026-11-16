import math
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Bool, String
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import (
    ComputePathToPose,
    NavigateToPose,
    Spin
)
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class CoverageExplorer(Node):

    def __init__(self):
        super().__init__('coverage_explorer')

        # 설정 파일 파라미터
        defaults = {
            'map_topic': '/map',
            'costmap_topic': '/global_costmap/costmap',
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'waypoint_spacing': 0.8,
            'min_obstacle_clearance': 0.3,
            'max_costmap_value': 30,
            'min_goal_distance': 0.4,
            'map_refresh_interval': 5.0,
            'visited_radius': 0.4,
            'failed_radius': 0.4,
            'max_goal_retries': 2,
            'distance_weight': 1.0,
            'coverage_weight': 2.0,
            'enable_scan': True,
            'scan_angle': 6.283185,
            'spin_timeout': 40.0,
            'navigation_timeout': 120.0,
            'planning_timeout': 10.0,
            'control_period': 0.5
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(
                self, name,
                self.get_parameter(name).value
            )

        self.map_msg = None
        self.costmap_msg = None
        self.waypoints = []

        # 탐색 기록
        self.visited_points = []
        self.failed_points = []
        self.next_waypoint = None

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # SLAM 지도 구독
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos
        )

        # Global Costmap 구독
        self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            qos
        )

        # 로봇 위치 확인
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        # RViz 시각화
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/escape_room/coverage_waypoints',
            qos
        )

        # Nav2 Action Clients
        self.planner_client = ActionClient(
            self, ComputePathToPose,
            '/compute_path_to_pose'
        )

        self.nav_client = ActionClient(
            self, NavigateToPose,
            '/navigate_to_pose'
        )

        self.spin_client = ActionClient(
            self, Spin, '/spin'
        )

        # 탐색 실행 상태
        self.enabled = False
        self.search_target = None
        self.active_mode = None
        self.state = None

        self.current_goal = None
        self.action_kind = None
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None

        self.action_start = 0.0
        self.action_timeout = 0.0
        self.cancel_reason = None

        self.status_pub = self.create_publisher(
            String,
            '/escape_room/coverage_status',
            qos
        )

        self.create_subscription(
            Bool,
            '/escape_room/coverage_resume',
            self.resume_callback,
            10
        )

        self.create_subscription(
            String,
            '/escape_room/active_control_mode',
            self.mode_callback,
            qos
        )

        self.create_timer(
            self.control_period,
            self.control_loop
        )

        # 탐색 대상 관리
        self.target_ack_pub = self.create_publisher(
            String,
            '/escape_room/coverage_target_ack',
            qos
        )

        self.create_subscription(
            String,
            '/escape_room/coverage_target',
            self.target_callback,
            10
        )

        self.set_state('IDLE')

        self.create_timer(
            self.map_refresh_interval,
            self.refresh_waypoints
        )

        self.get_logger().info(
            'Coverage Explorer started'
        )

    def map_callback(self, msg):
        self.map_msg = msg

    def costmap_callback(self, msg):
        self.costmap_msg = msg

    def get_robot_position(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time()
            )

            t = transform.transform.translation

            return t.x, t.y

        except TransformException:
            return None

    def grid_yaw(self, msg):

        q = msg.info.origin.orientation

        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )

    # 지도 셀 좌표 -> 실제 지도 좌표
    def grid_to_world(self, gx, gy, msg):

        resolution = msg.info.resolution
        origin = msg.info.origin.position
        yaw = self.grid_yaw(msg)

        lx = (gx + 0.5) * resolution
        ly = (gy + 0.5) * resolution

        x = origin.x + math.cos(yaw)*lx - math.sin(yaw)*ly
        y = origin.y + math.sin(yaw)*lx + math.cos(yaw)*ly

        return x, y

    # 실제 지도 좌표 -> 지도 셀 좌표
    def world_to_grid(self, x, y, msg):

        origin = msg.info.origin.position
        resolution = msg.info.resolution
        yaw = self.grid_yaw(msg)

        dx = x - origin.x
        dy = y - origin.y

        lx = math.cos(yaw)*dx + math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx + math.cos(yaw)*dy

        return (
            math.floor(lx / resolution),
            math.floor(ly / resolution)
        )

    # 장애물 및 미탐색 영역 확인
    def has_clearance(self, grid, gx, gy, radius):

        height, width = grid.shape

        if (
            gx-radius < 0
            or gy-radius < 0
            or gx+radius >= width
            or gy+radius >= height
        ):
            return False

        area = grid[
            gy-radius:gy+radius+1,
            gx-radius:gx+radius+1
        ]

        # 주변 영역이 모두 알려진 자유 공간인지 확인
        return bool(np.all(area == 0))

    def generate_waypoints(self, robot_position):

        msg = self.map_msg
        cost_msg = self.costmap_msg

        grid = np.asarray(msg.data).reshape(
            msg.info.height,
            msg.info.width
        )

        costmap = np.asarray(cost_msg.data).reshape(
            cost_msg.info.height,
            cost_msg.info.width
        )

        spacing = max(
            1,
            round(
                self.waypoint_spacing
                / msg.info.resolution
            )
        )

        clearance = math.ceil(
            self.min_obstacle_clearance
            / msg.info.resolution
        )

        candidates = []

        for gy in range(0, msg.info.height, spacing):

            for gx in range(0, msg.info.width, spacing):

                # 자유 공간이 아니면 제외
                if grid[gy, gx] != 0:
                    continue

                if not self.has_clearance(
                    grid, gx, gy, clearance
                ):
                    continue

                x, y = self.grid_to_world(gx, gy, msg)

                cx, cy = self.world_to_grid(
                    x, y, cost_msg
                )

                # Costmap 범위 확인
                if not (
                    0 <= cx < cost_msg.info.width
                    and 0 <= cy < cost_msg.info.height
                ):
                    continue

                cost = costmap[cy, cx]

                # 미탐색 및 충돌 위험 영역 제외
                if cost < 0 or cost > self.max_costmap_value:
                    continue

                distance = math.hypot(
                    x-robot_position[0],
                    y-robot_position[1]
                )

                if distance < self.min_goal_distance:
                    continue

                candidates.append((x, y))

        # 현재 로봇 위치에서 가까운 후보 우선
        candidates.sort(
            key=lambda p: math.hypot(
                p[0]-robot_position[0],
                p[1]-robot_position[1]
            )
        )

        return candidates

    # 두 좌표 사이의 거리
    def point_distance(self, a, b):

        return math.hypot(
            a[0] - b[0],
            a[1] - b[1]
        )

    # 이미 탐색한 지점인지 확인
    def is_visited(self, point):

        return any(
            self.point_distance(point, visited)
            <= self.visited_radius
            for visited in self.visited_points
        )

    # 이동 실패 횟수 확인
    def failure_count(self, point):

        return max(
            (
                count
                for failed, count in self.failed_points
                if self.point_distance(point, failed)
                <= self.failed_radius
            ),
            default=0
        )

    def is_failed(self, point):

        return (
            self.failure_count(point)
            >= self.max_goal_retries
        )

    # 탐색 완료 기록
    def record_visit(self, point):

        if not self.is_visited(point):

            self.visited_points.append(point)

            self.get_logger().info(
                f'WAYPOINT_VISITED: {point}'
            )

    # 이동 실패 기록
    def record_failure(self, point):

        for i, (failed, count) in enumerate(
            self.failed_points
        ):

            if (
                self.point_distance(point, failed)
                <= self.failed_radius
            ):

                self.failed_points[i] = (
                    failed, count + 1
                )

                return

        self.failed_points.append((point, 1))

    # 주변 미방문 후보를 이용한 탐색 가치
    def coverage_gain(self, point):

        radius = self.waypoint_spacing * 1.5

        nearby = [
            p for p in self.waypoints
            if self.point_distance(point, p) <= radius
        ]

        if not nearby:
            return 0.0

        unvisited = sum(
            1 for p in nearby
            if not self.is_visited(p)
            and not self.is_failed(p)
        )

        return unvisited / len(nearby)

    # 후보별 탐색 점수 계산
    def waypoint_score(self, point, robot_position):

        distance = self.point_distance(
            point, robot_position
        )

        gain = self.coverage_gain(point)

        return (
            self.coverage_weight * gain
            - self.distance_weight * distance
        )

    # 다음 탐색 목적지 선정
    def select_next_waypoint(self, robot_position):

        available = [
            point for point in self.waypoints
            if not self.is_visited(point)
            and not self.is_failed(point)
        ]

        if not available:
            return None

        return max(
            available,
            key=lambda point:
                self.waypoint_score(
                    point, robot_position
                )
        )

    def sim_now(self):

        return (
            self.get_clock().now().nanoseconds / 1e9
        )

    def set_state(self, state):

        if self.state != state:

            self.state = state

            self.status_pub.publish(
                String(data=state)
            )

            self.get_logger().info(
                f'COVERAGE_STATE: {state}'
            )

    def target_callback(self, msg):

        target = msg.data.strip()

        if not target:
            return

        # 탐색 중에는 기록을 초기화하지 않음
        if self.enabled or self.action_kind is not None:
            self.get_logger().warning(
                'TARGET_CHANGE_REJECTED: coverage is active'
            )
            return

        if target != self.search_target:

            self.search_target = target
            self.visited_points.clear()
            self.failed_points.clear()

            self.current_goal = None
            self.next_waypoint = None

            self.set_state('PAUSED')

            self.get_logger().info(
                f'COVERAGE_TARGET: {target}'
            )

            self.get_logger().info(
                'COVERAGE_HISTORY_RESET'
            )

            self.refresh_waypoints()

        self.target_ack_pub.publish(
            String(data=target)
        )

    def resume_callback(self, msg):

        if not msg.data:

            self.enabled = False
            self.request_cancel('PAUSED')
            return

        if self.enabled:
            return

        if self.action_kind is not None:
            return

        self.enabled = True
        self.set_state('SELECT_GOAL')

    def mode_callback(self, msg):

        self.active_mode = msg.data

        if self.active_mode != 'NAVIGATION':
            self.enabled = False
            self.request_cancel('PAUSED')

    def make_pose(self, point):

        pose = PoseStamped()

        pose.header.frame_id = self.global_frame
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])

        robot = self.get_robot_position()

        yaw = 0.0

        if robot is not None:

            yaw = math.atan2(
                point[1] - robot[1],
                point[0] - robot[0]
            )

        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)

        return pose

    def send_action(
        self, client, goal, kind, timeout
    ):

        if not client.server_is_ready():
            return False

        self.action_kind = kind
        self.action_start = self.sim_now()
        self.action_timeout = timeout
        self.cancel_reason = None

        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None

        self.goal_future = (
            client.send_goal_async(goal)
        )

        self.set_state(kind)

        return True

    def start_planning(self):

        goal = ComputePathToPose.Goal()

        goal.goal = self.make_pose(
            self.current_goal
        )

        goal.use_start = False

        self.send_action(
            self.planner_client,
            goal,
            'PLANNING',
            self.planning_timeout
        )

    def start_navigation(self):

        goal = NavigateToPose.Goal()

        goal.pose = self.make_pose(
            self.current_goal
        )

        self.send_action(
            self.nav_client,
            goal,
            'NAVIGATING',
            self.navigation_timeout
        )

    def start_spin(self):

        goal = Spin.Goal()

        goal.target_yaw = float(self.scan_angle)

        goal.time_allowance.sec = int(
            self.spin_timeout
        )

        self.send_action(
            self.spin_client,
            goal,
            'SCANNING',
            self.spin_timeout + 5.0
        )

    def request_cancel(self, reason):

        if self.action_kind is None:

            if reason == 'PAUSED':
                self.current_goal = None
                self.next_waypoint = None
                self.set_state('PAUSED')

            return

        if reason == 'PAUSED':
            self.cancel_reason = 'PAUSED'

        if self.cancel_reason is None:
            self.cancel_reason = reason

        self.set_state('CANCELING')

        if (
            self.goal_handle is not None
            and self.cancel_future is None
        ):

            self.cancel_future = (
                self.goal_handle.cancel_goal_async()
            )

    def clear_action(self):

        self.action_kind = None
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None
        self.cancel_reason = None

    def finish_waypoint(self):

        self.record_visit(self.current_goal)

        self.current_goal = None

        self.set_state('SELECT_GOAL')

    def fail_waypoint(self, reason):

        self.get_logger().warning(
            f'WAYPOINT_FAILED: '
            f'{self.current_goal} | {reason}'
        )

        if self.current_goal is not None:
            self.record_failure(self.current_goal)

        self.current_goal = None

        self.set_state('SELECT_GOAL')

    def finish_action(self, response):

        kind = self.action_kind
        reason = self.cancel_reason

        self.clear_action()

        if reason == 'PAUSED':

            self.current_goal = None
            self.set_state('PAUSED')
            return

        if reason == 'TIMEOUT':

            self.fail_waypoint('ACTION_TIMEOUT')
            return

        success = (
            response.status
            == GoalStatus.STATUS_SUCCEEDED
            and response.result.error_code == 0
        )

        if not success:

            self.fail_waypoint(
                f'{kind}: '
                f'status={response.status}, '
                f'error_code={response.result.error_code}, '
                f'message={response.result.error_msg}'
            )
            return

        if kind == 'PLANNING':

            if not response.result.path.poses:

                self.fail_waypoint('EMPTY_PATH')
                return

            self.set_state('READY_TO_NAVIGATE')

        elif kind == 'NAVIGATING':

            self.get_logger().info(
                f'WAYPOINT_REACHED: '
                f'{self.current_goal}'
            )

            if self.enable_scan:
                self.set_state('READY_TO_SCAN')
            else:
                self.finish_waypoint()

        elif kind == 'SCANNING':

            self.get_logger().info(
                'SCAN_COMPLETED'
            )

            self.finish_waypoint()

    def poll_action(self):

        if (
            self.sim_now() - self.action_start
            > self.action_timeout
        ):
            self.request_cancel('TIMEOUT')

        if self.goal_handle is None:

            if not self.goal_future.done():
                return

            self.goal_handle = (
                self.goal_future.result()
            )

            if not self.goal_handle.accepted:

                reason = self.cancel_reason
                self.clear_action()

                if reason == 'PAUSED':
                    self.current_goal = None
                    self.set_state('PAUSED')
                else:
                    self.fail_waypoint(
                        'GOAL_REJECTED'
                    )

                return

            self.result_future = (
                self.goal_handle.get_result_async()
            )

        if self.result_future.done():

            self.finish_action(
                self.result_future.result()
            )

            return

        if self.cancel_reason is not None:

            self.request_cancel(
                self.cancel_reason
            )

    def control_loop(self):

        # 이동 중에도 중단 명령 및 결과 확인
        if self.action_kind is not None:
            self.poll_action()
            return

        if not self.enabled:
            return

        if self.active_mode != 'NAVIGATION':
            return

        if (
            self.map_msg is None
            or self.costmap_msg is None
        ):
            return

        if not all([
            self.planner_client.server_is_ready(),
            self.nav_client.server_is_ready(),
            self.spin_client.server_is_ready()
        ]):
            self.set_state('WAITING_FOR_NAV2')
            return

        robot = self.get_robot_position()

        if robot is None:
            return

        if self.state == 'READY_TO_NAVIGATE':

            self.start_navigation()
            return

        if self.state == 'READY_TO_SCAN':

            self.start_spin()
            return

        if self.current_goal is None:

            self.waypoints = (
                self.generate_waypoints(robot)
            )

            self.current_goal = (
                self.select_next_waypoint(robot)
            )

            if self.current_goal is None:

                self.enabled = False
                self.set_state('COMPLETE')

                self.get_logger().info(
                    'COVERAGE_SEARCH_FINISHED'
                )

                return

            self.get_logger().info(
                f'GOAL_SELECTED: '
                f'{self.current_goal}'
            )

        self.start_planning()

    def publish_markers(self):

        marker = Marker()

        marker.header.frame_id = self.global_frame
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'coverage_waypoints'
        marker.id = 0

        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0

        for x, y in self.waypoints:

            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.1

            marker.points.append(point)

            color = ColorRGBA()
            color.a = 1.0

            if self.is_visited((x, y)):
                # 초록색: 탐색 완료
                color.g = 0.8

            elif self.is_failed((x, y)):
                # 빨간색: 이동 실패
                color.r = 1.0

            elif (x, y) == self.next_waypoint:
                # 노란색: 다음 탐색 목적지
                color.r = 1.0
                color.g = 0.85

            else:
                # 하늘색: 미방문 후보
                color.g = 0.8
                color.b = 1.0

            marker.colors.append(color)

        markers = MarkerArray()
        markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def refresh_waypoints(self):

        if self.map_msg is None or self.costmap_msg is None:

            self.get_logger().info(
                'WAITING_FOR_MAP'
            )
            return

        if (
            self.map_msg.header.frame_id != self.global_frame
            or self.costmap_msg.header.frame_id != self.global_frame
        ):
            self.get_logger().warning('MAP_FRAME_MISMATCH')
            return

        robot_position = self.get_robot_position()

        if robot_position is None:

            self.get_logger().info(
                'WAITING_FOR_ROBOT_POSE'
            )
            return

        self.waypoints = self.generate_waypoints(
            robot_position
        )

        self.next_waypoint = (
            self.current_goal
            if self.current_goal is not None
            else self.select_next_waypoint(
                robot_position
            )
        )

        # 현재 목적지가 최소 거리 조건으로
        # 후보에서 사라져도 시각화 유지
        if (
            self.current_goal is not None
            and self.current_goal not in self.waypoints
        ):
            self.waypoints.append(self.current_goal)

        self.publish_markers()

        if self.next_waypoint is not None:

            x, y = self.next_waypoint

            score = self.waypoint_score(
                self.next_waypoint,
                robot_position
            )

            self.get_logger().info(
                f'NEXT_WAYPOINT: '
                f'x={x:.2f}, y={y:.2f} | '
                f'Score={score:.3f}'
            )

        self.get_logger().info(
            f'VISITED: {len(self.visited_points)} | '
            f'FAILED: {len(self.failed_points)}'
        )

        self.get_logger().info(
            f'WAYPOINTS_GENERATED: {len(self.waypoints)}'
        )


def main(args=None):

    rclpy.init(args=args)
    node = CoverageExplorer()

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
