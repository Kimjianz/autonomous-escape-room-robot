import rclpy
import subprocess
import sys
import signal
import time
import threading
import queue
import json

from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from explore_lite_msgs.msg import ExploreStatus
from action_msgs.msg import GoalStatusArray, GoalStatus
from rclpy.qos import qos_profile_sensor_data


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.script_dir = Path(__file__).resolve().parent

        self.processes = {}
        self.log_queue = queue.Queue()
        self.received_logs = set()

        self.state = None
        self.state_start = time.monotonic()
        self.stage_timeout = 900.0

        self.key_acquired = False
        self.key_removed = False
        self.door_reached = False
        self.door_opened = False
        self.escape_success = False

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 각 임무의 완료 상태 구독
        for name in [
            'key_acquired',
            'key_removed',
            'door_reached',
            'door_opened',
            'escape_success'
        ]:
            self.create_subscription(
                Bool,
                f'/escape_room/{name}',
                lambda msg, name=name:
                    self.update_status(name, msg),
                qos
            )

        # 현재 임무 상태 발행
        self.state_pub = self.create_publisher(
            String,
            '/escape_room/mission_state',
            qos
        )

        # 자율 탐색 및 이동 제어권 관리
        self.mode_pub = self.create_publisher(
            String, '/escape_room/control_mode', 10
        )

        self.explore_pub = self.create_publisher(
            Bool, '/explore/resume', 10
        )

        # Hybrid Exploration
        self.coverage_pub = self.create_publisher(
            Bool, '/escape_room/coverage_resume', 10
        )

        self.target_pub = self.create_publisher(
            String, '/escape_room/coverage_target', 10
        )

        self.frontier_status = None
        self.coverage_status = None
        self.coverage_target_ack = None

        self.search_phase = 'PREPARE'
        self.search_timeout = 600.0
        self.request_time = -100.0

        self.pause_since = None
        self.pause_last = -100.0
        self.quiet_since = None

        self.create_subscription(
            ExploreStatus,
            '/explore/status',
            self.frontier_callback,
            qos
        )

        self.create_subscription(
            String,
            '/escape_room/coverage_status',
            self.coverage_callback,
            qos
        )

        self.create_subscription(
            String,
            '/escape_room/coverage_target_ack',
            self.target_ack_callback,
            qos
        )

        # Nav2 Action 실행 상태 확인
        self.action_topics = [
            '/navigate_to_pose/_action/status',
            '/compute_path_to_pose/_action/status',
            '/spin/_action/status'
        ]

        self.action_busy = {
            topic: False for topic in self.action_topics
        }

        for topic in self.action_topics:
            self.create_subscription(
                GoalStatusArray,
                topic,
                lambda msg, topic=topic:
                    self.action_status_callback(topic, msg),
                qos
            )

        self.active_mode = None
        self.odom = None
        self.odom_time = -1.0
        self.detect_count = 0
        self.detect_last = -1.0
        self.last_seen = {}
        self.search_resumed = False
        self.required_detections = 3
        self.lost_timeout = 4.0

        self.create_subscription(
            String,
            '/escape_room/active_control_mode',
            self.mode_callback,
            qos
        )

        self.create_subscription(
            String,
            '/escape_room/detections',
            self.detection_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_timer(0.5, self.control_loop)

        self.get_logger().info('Mission Manager started')

        self.set_state('START')

    def update_status(self, name, msg):
        setattr(self, name, msg.data)

    def set_state(self, state):

        if self.state != state:

            self.state = state
            self.state_start = time.monotonic()
            self.sim_state_start = self.sim_now()
            self.detect_count = 0
            self.detect_last = -1.0
            self.search_resumed = False
            self.search_phase = 'PREPARE'
            self.request_time = -100.0

            self.state_pub.publish(
                String(data=state)
            )

            self.get_logger().info(
                f'MISSION_STATE: {state}'
            )

    # 실행 중인 노드 로그 수집
    def read_logs(self, name, process):

        for line in process.stdout:
            self.log_queue.put(
                (name, line.rstrip())
            )

    def collect_logs(self):

        while True:

            try:
                name, line = self.log_queue.get_nowait()
            except queue.Empty:
                break

            print(f'[{name}] {line}', flush=True)

            self.received_logs.add((name, line))

    def has_log(self, name, keyword):

        return any(
            process_name == name and keyword in line
            for process_name, line in self.received_logs
        )

    # 기존 Python 노드 자동 실행
    def start_node(self, name, sim_time=False):

        if name in self.processes:
            return

        command = [
            sys.executable,
            '-u',
            str(self.script_dir / f'{name}.py')
        ]

        if sim_time:
            command += [
                '--ros-args',
                '-p',
                'use_sim_time:=true'
            ]

        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True
        )

        self.processes[name] = process

        threading.Thread(
            target=self.read_logs,
            args=(name, process),
            daemon=True
        ).start()

        self.get_logger().info(
            f'NODE_STARTED: {name}'
        )

    # 기존 Python 노드 자동 종료
    def stop_node(self, name):

        process = self.processes.get(name)

        if process is None:
            return

        if process.poll() is None:

            process.send_signal(signal.SIGINT)

            try:
                process.wait(timeout=5)

            except subprocess.TimeoutExpired:

                process.terminate()

                try:
                    process.wait(timeout=3)

                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

        del self.processes[name]

        self.get_logger().info(
            f'NODE_STOPPED: {name}'
        )

    def stop_all(self):

        for name in reversed(list(self.processes)):
            self.stop_node(name)

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def mode_callback(self, msg):
        self.active_mode = msg.data

    def odom_callback(self, msg):
        self.odom = msg.twist.twist
        self.odom_time = self.sim_now()

    def detection_callback(self, msg):

        try:
            detections = json.loads(msg.data)
            labels = {
                item['label'] for item in detections
            }
        except (ValueError, TypeError, KeyError):
            return

        now = self.sim_now()

        for label in labels:
            self.last_seen[label] = now

        target = {
            'SEARCH_KEY': 'KEY',
            'SEARCH_DOOR': 'DOOR'
        }.get(self.state)

        if target is None:
            return

        if target in labels:

            if now - self.detect_last > 2.0:
                self.detect_count = 0

            self.detect_count += 1
            self.detect_last = now

        else:
            self.detect_count = 0

    def frontier_callback(self, msg):
        self.frontier_status = msg.status

    def coverage_callback(self, msg):
        self.coverage_status = msg.data

    def target_ack_callback(self, msg):
        self.coverage_target_ack = msg.data

    def action_status_callback(self, topic, msg):

        self.action_busy[topic] = any(
            goal.status in [
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING
            ]
            for goal in msg.status_list
        )

    def set_mode(self, mode):

        if mode != 'STOP':
            self.pause_since = None
            self.quiet_since = None

        self.mode_pub.publish(String(data=mode))

    def set_exploration(self, enabled):
        self.explore_pub.publish(Bool(data=enabled))

    def set_coverage(self, enabled):
        self.coverage_pub.publish(Bool(data=enabled))

    def set_search_phase(self, phase):

        self.search_phase = phase
        self.request_time = -100.0

        self.get_logger().info(
            f'SEARCH_PHASE: {phase}'
        )

    def pause_motion(self):

        now = self.sim_now()

        if self.pause_since is None:
            self.pause_since = now
            self.quiet_since = None
            self.pause_last = -100.0

        # 중단 명령은 응답을 확인할 때까지 재전송
        if now - self.pause_last >= 1.0:

            self.set_mode('STOP')
            self.set_exploration(False)
            self.set_coverage(False)

            self.pause_last = now

    def stopped(self):

        self.pause_motion()
        now = self.sim_now()

        ready = (
            self.active_mode == 'STOP'
            and self.frontier_status in [
                'exploration_paused',
                'exploration_complete'
            ]
            and self.coverage_status in [
                'IDLE', 'PAUSED', 'COMPLETE'
            ]
            and all(
                self.count_publishers(topic) > 0
                for topic in self.action_topics
            )
            and not any(self.action_busy.values())
            and self.odom is not None
            and 0.0 <= now - self.odom_time <= 2.0
            and abs(self.odom.linear.x) < 0.02
            and abs(self.odom.linear.y) < 0.02
            and abs(self.odom.angular.z) < 0.02
        )

        if not ready:
            self.quiet_since = None
            return False

        if self.quiet_since is None:
            self.quiet_since = now
            return False

        return now - self.quiet_since >= 1.0

    def search_step(self, target, next_state):

        now = self.sim_now()

        # 어떤 탐색 방식에서든 목표 객체 발견 시 중단
        if (
            self.detect_count >= self.required_detections
            and now - self.detect_last <= 2.0
        ):

            self.get_logger().info(
                f'TARGET_FOUND: {target} | {self.search_phase}'
            )

            self.pause_motion()
            self.set_state(next_state)
            return

        # 1. 기존 탐색 중단 확인
        if self.search_phase == 'PREPARE':

            if self.stopped():

                self.coverage_target_ack = None
                self.set_search_phase('SET_TARGET')

            return

        # 2. 탐색 대상 설정
        if self.search_phase == 'SET_TARGET':

            if self.coverage_target_ack == target:
                self.set_search_phase('FRONTIER_ARM')
                return

            if now - self.request_time >= 2.0:

                self.target_pub.publish(
                    String(data=target)
                )

                self.request_time = now

            return

        # 3. Frontier 탐색 시작
        if self.search_phase == 'FRONTIER_ARM':

            if self.active_mode != 'NAVIGATION':
                self.set_mode('NAVIGATION')
                return

            self.frontier_status = None

            self.set_exploration(True)
            self.set_search_phase('FRONTIER')

            self.request_time = now
            return

        # 4. Frontier 탐색 진행 및 완료 확인
        if self.search_phase == 'FRONTIER':

            if self.frontier_status == 'exploration_complete':

                self.get_logger().info(
                    'FRONTIER_EXHAUSTED: switching to coverage'
                )

                self.pause_motion()
                self.set_search_phase('SWITCH_TO_COVERAGE')
                return

            if (
                self.frontier_status is None
                and now - self.request_time >= 3.0
            ):
                self.set_exploration(True)
                self.request_time = now

            return

        # 5. Frontier 중단 및 실제 정지 확인
        if self.search_phase == 'SWITCH_TO_COVERAGE':

            if self.stopped():
                self.set_search_phase('COVERAGE_ARM')

            return

        # 6. Coverage 탐색 시작
        if self.search_phase == 'COVERAGE_ARM':

            if self.active_mode != 'NAVIGATION':
                self.set_mode('NAVIGATION')
                return

            self.coverage_status = None

            self.set_coverage(True)
            self.set_search_phase('COVERAGE')

            self.request_time = now
            return

        # 7. Coverage 탐색 진행
        if self.search_phase == 'COVERAGE':

            if self.coverage_status == 'COMPLETE':

                self.fail(
                    f'TARGET_NOT_FOUND: {target}'
                )
                return

            if (
                self.coverage_status is None
                and now - self.request_time >= 3.0
            ):
                self.set_coverage(True)
                self.request_time = now

    def fail(self, reason):

        self.pause_motion()
        self.get_logger().error(reason)
        self.set_state('MISSION_FAILED')
        self.stop_all()

    def control_loop(self):

        self.collect_logs()

        if self.state in [
            'ESCAPE_SUCCESS',
            'MISSION_FAILED'
        ]:
            self.pause_motion()
            return

        for name, process in self.processes.items():

            if process.poll() is not None:
                self.fail(f'Unexpected node exit: {name}')
                return

        # 시뮬레이션 시간 기준 단계 제한
        if self.state == 'START' and self.sim_state_start <= 0.0:
            self.sim_state_start = self.sim_now()

        timeout = (
            self.search_timeout
            if self.state in ['SEARCH_KEY', 'SEARCH_DOOR']
            else 180.0
        )

        if self.sim_now() - self.sim_state_start > timeout:
            self.fail(f'Stage timeout: {self.state}')
            return

        # 1. 미션 실행 준비
        if self.state == 'START':

            if self.count_publishers('/cmd_vel_approach') != 0:
                self.fail('Another approach node exists')
                return

            if (
                self.mode_pub.get_subscription_count() == 0
                or self.explore_pub.get_subscription_count() == 0
                or self.coverage_pub.get_subscription_count() == 0
                or self.target_pub.get_subscription_count() == 0
                or self.count_publishers(
                    '/escape_room/detections'
                ) == 0
            ):
                return

            if any([
                self.key_acquired,
                self.key_removed,
                self.door_reached,
                self.door_opened,
                self.escape_success
            ]):
                self.fail('Previous mission status detected')
                return

            if self.stopped():
                self.set_state('SEARCH_KEY')

        # 2. 열쇠 자율 탐색
        elif self.state == 'SEARCH_KEY':

            self.search_step('KEY', 'KEY_STOP')

        # 3. 정지 후 열쇠 접근 시작
        elif self.state == 'KEY_STOP':

            if self.stopped():

                self.start_node(
                    'key_approach',
                    sim_time=True
                )

                self.set_state('KEY_ACQUISITION')
                self.set_mode('APPROACH')

        # 4. 열쇠 획득 확인
        elif self.state == 'KEY_ACQUISITION':

            if self.key_acquired:

                self.pause_motion()
                self.set_state('KEY_ACQUIRED_STOP')
                return

            if self.active_mode != 'APPROACH':
                self.set_mode('APPROACH')

            last_seen = max(
                self.last_seen.get('KEY', -1.0),
                self.sim_state_start
            )

            if self.sim_now() - last_seen > self.lost_timeout:

                self.get_logger().warning(
                    'KEY_LOST: restarting exploration'
                )

                self.pause_motion()
                self.set_state('KEY_RETRY')

        # 열쇠를 놓친 경우 재탐색
        elif self.state == 'KEY_RETRY':

            if self.stopped():

                self.stop_node('key_approach')

                self.key_acquired = False

                self.set_state('SEARCH_KEY')

        # 5. 열쇠 제거
        elif self.state == 'KEY_ACQUIRED_STOP':

            if self.stopped():

                self.start_node('key_remover')
                self.set_state('KEY_REMOVAL')

        # 6. 문 탐색 시작
        elif self.state == 'KEY_REMOVAL':

            if self.key_removed:

                self.set_state('SEARCH_DOOR')

        # 7. 문 자율 탐색
        elif self.state == 'SEARCH_DOOR':

            self.search_step('DOOR', 'DOOR_STOP')

        # 8. 정지 후 문 접근 시작
        elif self.state == 'DOOR_STOP':

            if self.stopped():

                self.start_node(
                    'door_approach',
                    sim_time=True
                )

                self.set_state('KEY_TO_DOOR')

        # 9. 열쇠 접근 노드에서 문 접근 노드로 전환
        elif self.state == 'KEY_TO_DOOR':

            if (
                self.has_log(
                    'door_approach',
                    'KEY_POSSESSION_CONFIRMED'
                )
                and self.has_log(
                    'door_approach',
                    'KEY_REMOVAL_CONFIRMED'
                )
                and self.has_log(
                    'door_approach',
                    'WAITING_FOR_CONTROL'
                )
            ):

                self.stop_node('key_approach')

                self.set_state('DOOR_APPROACH')
                self.set_mode('APPROACH')

        # 10. 문 접근 완료
        elif self.state == 'DOOR_APPROACH':

            if self.door_reached:

                self.pause_motion()
                self.set_state('DOOR_REACHED_STOP')

            elif self.active_mode != 'APPROACH':
                self.set_mode('APPROACH')

        # 11. 문 개방
        elif self.state == 'DOOR_REACHED_STOP':

            if self.stopped():

                self.start_node('door_opener')
                self.set_state('DOOR_OPENING')

        # 12. 문 개방 완료 후 탈출구 접근
        elif self.state == 'DOOR_OPENING':

            if self.door_opened:

                self.start_node(
                    'exit_approach',
                    sim_time=True
                )

                self.set_state('DOOR_TO_EXIT')

        # 13. 문 접근 노드에서 탈출구 접근 노드로 전환
        elif self.state == 'DOOR_TO_EXIT':

            if (
                self.has_log(
                    'exit_approach',
                    'DOOR_OPEN_CONFIRMED'
                )
                and self.has_log(
                    'exit_approach',
                    'WAITING_FOR_CONTROL'
                )
            ):

                self.stop_node('door_approach')

                self.set_state('EXIT_APPROACH')
                self.set_mode('APPROACH')

        # 14. 최종 탈출 성공
        elif self.state == 'EXIT_APPROACH':

            if self.escape_success:

                self.pause_motion()
                self.set_state('ESCAPE_SUCCESS')

                self.get_logger().info(
                    'LEVEL 1 MISSION COMPLETED'
                )

            elif self.active_mode != 'APPROACH':
                self.set_mode('APPROACH')


def main(args=None):

    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.pause_motion()

        node.stop_all()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
