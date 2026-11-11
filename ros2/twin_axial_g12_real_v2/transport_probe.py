"""Actual-process transport probe for state, Gazebo pose and MAS action channels.

This is deliberately not a string-only probe: separate Python processes use
the real ROS message types over the same environment as the runner.
"""
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOMAIN = '74'
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'


def _env():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = DOMAIN
    env['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    return env


def _write(path, data):
    Path(path).write_text(json.dumps(data), encoding='utf-8')


def _read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _state_payload():
    return {'sequence': 0, 'valid': True, 'elapsed_end': 0.05,
            'solver_state_ned_frd': {'t': 0.05, 'x': 0.0, 'y': 2.0, 'z': 30.0,
                                     'qr': 0.7071067811865476, 'qi': 0.0, 'qj': 0.0, 'qk': 0.7071067811865475,
                                     'u': 0.0, 'v': 0.0, 'w': 0.0, 'p': 0.0, 'q': 0.0, 'r': 0.0},
            'pitch_nose_up_rad': 0.0, 'watchdog_zero': False,
            'applied_commands': {'upper(T)': 0.0, 'lower(T)': 0.0}, 'error': None}


def _state_sub(out):
    import rclpy
    from std_msgs.msg import String
    rclpy.init(); node = rclpy.create_node('g12_state_probe_sub')
    got = []
    sub = node.create_subscription(String, '/probe/state', lambda m: got.append(len(m.data)), 20)
    end = time.monotonic() + 8
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    _write(out, {'received': len(got), 'lengths': got[:5], 'domain': rclpy.get_default_context().get_domain_id(), 'topics': node.get_topic_names_and_types(), 'passed': len(got) >= 25})
    node.destroy_node(); rclpy.shutdown()


def _state_pub(out):
    import rclpy
    from std_msgs.msg import String
    rclpy.init(); node = rclpy.create_node('g12_state_probe_pub')
    pub = node.create_publisher(String, '/probe/state', 20)
    payload = json.dumps(_state_payload(), allow_nan=False)  # 374 B compact state
    time.sleep(1)
    for _ in range(30):
        msg = String(); msg.data = payload; pub.publish(msg); time.sleep(.05)
    _write(out, {'published': 30, 'payload_bytes': len(payload), 'domain': rclpy.get_default_context().get_domain_id(), 'passed': len(payload) < 1000})
    node.destroy_node(); rclpy.shutdown()


def _pose_sub(out):
    import rclpy
    from lotusim_msgs.msg import VesselPositionArray
    rclpy.init(); node = rclpy.create_node('g12_pose_probe_sub')
    got = []
    sub = node.create_subscription(VesselPositionArray, '/probe/pose', lambda m: got.append(len(m.vessels)), 20)
    end = time.monotonic() + 8
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    _write(out, {'received': len(got), 'first': got[:3], 'passed': len(got) >= 25})
    node.destroy_node(); rclpy.shutdown()


def _pose_pub(out):
    import rclpy
    from lotusim_msgs.msg import VesselPosition, VesselPositionArray
    rclpy.init(); node = rclpy.create_node('g12_pose_probe_pub')
    pub = node.create_publisher(VesselPositionArray, '/probe/pose', 20)
    time.sleep(1)
    for _ in range(30):
        msg = VesselPositionArray(); msg.header.frame_id = 'probe'
        vessel = VesselPosition(); vessel.vessel_name = 'probe'; vessel.model_name = 'probe'
        vessel.pose.position.x = 1.0; vessel.pose.orientation.w = 1.0
        msg.vessels = [vessel]; pub.publish(msg)
        time.sleep(.05)
    _write(out, {'published': 30, 'passed': True})
    node.destroy_node(); rclpy.shutdown()


def _action_server(out):
    import rclpy
    from rclpy.action import ActionServer
    from lotusim_msgs.action import MASCmdArray
    rclpy.init(); node = rclpy.create_node('g12_action_probe_server')
    served = []
    def execute(goal_handle):
        result = MASCmdArray.Result()
        result.result = [True]; result.name = ['probe']; result.entity = [0]
        goal_handle.succeed(); served.append(True)
        return result
    server = ActionServer(node, MASCmdArray, 'probe_mas', execute)
    import threading
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    end = time.monotonic() + 30
    while time.monotonic() < end and not served:
        time.sleep(.1)
    _write(out, {'served': bool(served), 'passed': bool(served)})
    executor.shutdown(); spinner.join(timeout=2)
    server.destroy(); node.destroy_node(); rclpy.shutdown()


def _action_client(out):
    import rclpy
    from rclpy.action import ActionClient
    from lotusim_msgs.action import MASCmdArray
    rclpy.init(); node = rclpy.create_node('g12_action_probe_client')
    client = ActionClient(node, MASCmdArray, 'probe_mas')
    time.sleep(1)
    if not client.wait_for_server(timeout_sec=20):
        _write(out, {'passed': False, 'error': 'server unavailable'})
        node.destroy_node(); rclpy.shutdown(); return
    goal = MASCmdArray.Goal(); goal.cmd = []
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8)
    if not future.done() or not future.result().accepted:
        _write(out, {'passed': False, 'error': 'goal not accepted'})
    else:
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=8)
        ok = result_future.done() and result_future.result().status == 4 and all(result_future.result().result.result)
        _write(out, {'passed': bool(ok), 'status': result_future.result().status if result_future.done() else None})
    client.destroy(); node.destroy_node(); rclpy.shutdown()


def _pair(sub_role, pub_role, sub_out, pub_out):
    sub = subprocess.Popen([sys.executable, __file__, sub_role, str(sub_out)], env=None,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    time.sleep(1)
    pub = subprocess.Popen([sys.executable, __file__, pub_role, str(pub_out)], env=None,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        pub_rc = pub.wait(timeout=40)
    except subprocess.TimeoutExpired:
        pub.kill(); pub_rc = 'timeout'
    try:
        sub_rc = sub.wait(timeout=40)
    except subprocess.TimeoutExpired:
        sub.kill(); sub_rc = 'timeout'
    result = {'sub_returncode': sub_rc, 'pub_returncode': pub_rc,
              'sub_stderr': sub.stderr.read()[-500:] if sub.stderr else '',
              'pub_stderr': pub.stderr.read()[-500:] if pub.stderr else ''}
    return result


def run_all():
    base = Path(tempfile.mkdtemp(prefix='g12_transport_', dir=str(ROOT / 'artifacts/twin_axial_g12_real')))
    state = _pair('state-sub', 'state-pub', base/'state_sub.json', base/'state_pub.json')
    pose = _pair('pose-sub', 'pose-pub', base/'pose_sub.json', base/'pose_pub.json')
    action = _pair('action-server', 'action-client', base/'action_server.json', base/'action_client.json')
    def merge(pair, files):
        data = dict(pair)
        for key, name in files:
            path = base / name
            if path.exists():
                data[key] = _read(path)
        return data
    result = {'state': merge(state, (('sub', 'state_sub.json'), ('pub', 'state_pub.json'))),
              'pose': merge(pose, (('sub', 'pose_sub.json'), ('pub', 'pose_pub.json'))),
              'action': merge(action, (('server', 'action_server.json'), ('client', 'action_client.json'))),
              'directory': str(base)}
    result['passed'] = all(result[channel].get(first, {}).get('passed', False)
                           and result[channel].get(second, {}).get('passed', False)
                           for channel, first, second in
                           (('state', 'sub', 'pub'), ('pose', 'sub', 'pub'), ('action', 'server', 'client')))
    result['scope'] = 'Actual ROS message types in separate processes; no Gazebo, xdyn or control launch'
    return result


def main():
    role, out = sys.argv[1], sys.argv[2]
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    {'state-sub': _state_sub, 'state-pub': _state_pub, 'pose-sub': _pose_sub,
     'pose-pub': _pose_pub, 'action-server': _action_server,
     'action-client': _action_client}[role](out)


if __name__ == '__main__':
    main()
