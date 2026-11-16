import rclpy
import subprocess
import sys
import signal
import time
import threading
import queue

from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String


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

        self.create_timer(0.5, self.control_loop)

        self.get_logger().info('Mission Manager started')

        self.set_state('START')

    def update_status(self, name, msg):
        setattr(self, name, msg.data)

    def set_state(self, state):

        if self.state != state:

            self.state = state
            self.state_start = time.monotonic()

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

    def fail(self, reason):

        self.get_logger().error(reason)

        self.set_state('MISSION_FAILED')

        self.stop_all()

    # FSM 기반 임무 관리
    def control_loop(self):

        self.collect_logs()

        if self.state in [
            'ESCAPE_SUCCESS',
            'MISSION_FAILED'
        ]:
            return

        # 비정상 종료 확인
        for name, process in self.processes.items():

            if process.poll() is not None:
                self.fail(f'Unexpected node exit: {name}')
                return

        # 무한 대기 방지
        if (
            time.monotonic() - self.state_start
            > self.stage_timeout
        ):
            self.fail(f'Stage timeout: {self.state}')
            return

        # 1. 열쇠 자율 접근 시작
        if self.state == 'START':

            if self.count_publishers('/cmd_vel') != 0:
                self.fail('Another motion controller exists')
                return

            self.start_node(
                'key_approach',
                sim_time=True
            )

            self.set_state('KEY_ACQUISITION')

        # 2. 열쇠 획득 확인
        elif self.state == 'KEY_ACQUISITION':

            if self.key_acquired:

                self.start_node('key_remover')

                self.set_state('KEY_REMOVAL')

        # 3. 열쇠 제거 확인
        elif self.state == 'KEY_REMOVAL':

            if self.key_removed:

                self.start_node(
                    'door_approach',
                    sim_time=True
                )

                self.set_state('KEY_TO_DOOR')

        # 4. 이동 제어권 전환
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

        # 5. 문 접근 완료
        elif self.state == 'DOOR_APPROACH':

            if self.door_reached:

                self.start_node('door_opener')

                self.set_state('DOOR_OPENING')

        # 6. 문 개방 완료
        elif self.state == 'DOOR_OPENING':

            if self.door_opened:

                self.start_node(
                    'exit_approach',
                    sim_time=True
                )

                self.set_state('DOOR_TO_EXIT')

        # 7. 이동 제어권 전환
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

        # 8. 최종 탈출 성공
        elif self.state == 'EXIT_APPROACH':

            if self.escape_success:

                self.set_state('ESCAPE_SUCCESS')

                self.get_logger().info(
                    'LEVEL 1 MISSION COMPLETED'
                )


def main(args=None):

    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_all()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
