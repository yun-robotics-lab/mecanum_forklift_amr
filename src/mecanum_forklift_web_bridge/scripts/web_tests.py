#!/usr/bin/env python3
"""
Integration tests of the web layer against the running simulation (development tool).

Usage: web_tests.py [case numbers, comma separated]. Needs the simulation, navigation, vision,
docking, mission and web launches (dev_stack.sh start / start_mission / start_web).
Cases: 1 listeners, 2 auth + invalid requests, 3 missions over REST + event rate, 4 cancel over
REST, 5 mission manager down, 6 e-stop, 7 rosbridge allow-list, 8 video + dashboard, 9 e-stop
without the mission layer.
"""
import asyncio
import json
import math
import os
import subprocess
import sys
import time

import httpx
import rclpy
from geometry_msgs.msg import Twist
import websockets

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                'mecanum_forklift_task_manager', 'scripts'))
from mission_common import PAD, pallet_state, Rig  # noqa: E402

BASE = 'http://127.0.0.1:8000'
WS = 'ws://127.0.0.1:9090'
VIDEO = 'http://127.0.0.1:8080/stream?topic=/camera/image_raw&type=mjpeg&quality=50'
TOKEN = open(os.path.expanduser('~/.config/mecanum_forklift/web_token'), encoding='utf-8').read(
).strip()
AUTH = {'Authorization': f'Bearer {TOKEN}'}
STACK = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                     'mecanum_forklift_docking', 'scripts', 'dev_stack.sh')
GOOD = {'pick_dock_id': 'pallet_pick_1', 'drop_dock_id': 'pallet_drop_1'}


def call(method, path, body=None, headers=AUTH, raw=None):
    """One REST call; return the httpx response."""
    kwargs = {'headers': {**(headers or {}), 'Content-Type': 'application/json'}}
    if raw is not None:
        kwargs['content'] = raw
    elif body is not None:
        kwargs['content'] = json.dumps(body)
    return httpx.request(method, BASE + path, timeout=20, **kwargs)


def line(name, ok, detail=''):
    """Print one result line."""
    print(f'{name:44s} {"PASS" if ok else "FAIL"}  {detail}', flush=True)
    return ok


def wait_mission(mission_id, timeout=400):
    """Poll GET /missions/{id} until it is no longer active; return the record or None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        info = call('GET', f'/missions/{mission_id}').json()
        if not info.get('active'):
            return info
        time.sleep(1.0)
    return None


def event_rate(seconds):
    """Count Server-Sent Events over `seconds`; return events per second."""
    count = 0
    t0 = time.time()
    with httpx.stream('GET', BASE + '/events', headers=AUTH, timeout=seconds + 10) as resp:
        for text in resp.iter_text():
            count += text.count('data: ')
            if time.time() - t0 >= seconds:
                break
    return count / (time.time() - t0)


def listening(port):
    """Return the local addresses listening on a TCP port."""
    out = subprocess.run(['ss', '-ltn'], capture_output=True, text=True).stdout
    return [cols[3] for cols in (ln.split() for ln in out.splitlines()[1:])
            if cols[3].endswith(f':{port}')]


def case_listeners(rig):
    """REST, rosbridge and video listen on the configured ports, on 127.0.0.1 only."""
    got = {port: listening(port) for port in (8000, 9090, 8080)}
    ok = all(addrs == [f'127.0.0.1:{port}'] for port, addrs in got.items())
    return line('listeners (127.0.0.1 only)', ok, str(got))


def case_auth_and_invalid(rig):
    """Auth and the invalid-request matrix against the real server; nothing reaches the robot."""
    results = []
    for name, method, path, headers in (
            ('no token', 'GET', '/status', None), ('wrong token', 'GET', '/status',
                                                   {'Authorization': 'Bearer nope'}),
            ('no token POST /missions', 'POST', '/missions', None),
            ('no token POST /estop', 'POST', '/estop', None)):
        resp = call(method, path, GOOD if method == 'POST' else None, headers=headers)
        results.append(resp.status_code == 401)
    matrix = [
        ('not json', {'raw': 'nonsense'}, 400, 'INVALID_JSON'),
        ('missing drop dock', {'body': {'pick_dock_id': 'pallet_pick_1'}}, 400, 'INVALID_PAYLOAD'),
        ('unknown dock', {'body': {**GOOD, 'pick_dock_id': 'x'}}, 400, 'UNKNOWN_DOCK'),
        ('test-only dock hidden', {'body': {**GOOD, 'pick_dock_id': 'test_lift_out_of_range'}},
         400, 'UNKNOWN_DOCK'),
        ('drop dock as pick', {'body': {**GOOD, 'pick_dock_id': 'pallet_drop_1'}}, 400,
         'WRONG_DOCK_OPERATION'),
        ('bad mission id', {'body': {**GOOD, 'mission_id': '../x'}}, 400, 'INVALID_MISSION_ID'),
        ('return pose outside the map', {'body': {**GOOD, 'return_pose': {'x': 9, 'y': 0}}}, 400,
         'POSE_OUT_OF_BOUNDS'),
        ('velocity field', {'body': {**GOOD, 'cmd_vel': {'x': 1}}}, 400, 'INVALID_PAYLOAD'),
        ('oversized body', {'body': {**GOOD, 'pad': 'x' * 5000}}, 413, 'PAYLOAD_TOO_LARGE'),
    ]
    bad = []
    for name, req, status, code in matrix:
        resp = call('POST', '/missions', req.get('body'), raw=req.get('raw'))
        if resp.status_code != status or resp.json().get('error_code') != code:
            bad.append((name, resp.status_code, resp.text[:80]))
        time.sleep(7)                      # stay under the rate limit (10 per minute)
    status = call('GET', '/status').json()
    idle = status['mission']['state'] == 'IDLE' and not status['mission']['active']
    return line('auth + invalid requests (no side effect)', all(results) and not bad and idle,
                f'bad {bad}' if bad else f'{len(matrix)} invalid + 4 auth cases, robot idle')


def case_missions(rig, count=3):
    """Missions submitted over REST run to success; measure the event stream rate."""
    oks = []
    for k in range(count):
        rig.reset_world()
        resp = call('POST', '/missions', {**GOOD, 'mission_id': f'web-t{int(time.time())}-{k}',
                                          'return_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}})
        if resp.status_code != 202:
            oks.append(False)
            line(f'REST mission {k + 1}', False, resp.text[:120])
            continue
        mission_id = resp.json()['mission_id']
        rate = event_rate(20) if k == 0 else None
        info = wait_mission(mission_id)
        rig.spin(2.0)
        pal = pallet_state() or (0, 0, 0, 99)
        pad_cm = math.hypot(pal[0] - PAD[0], pal[1] - PAD[1]) * 100
        good = bool(info and info['success'] and pad_cm <= 8.0 and pal[3] <= 2.0)
        oks.append(good)
        extra = f'event rate {rate:.2f} Hz, ' if rate is not None else ''
        line(f'REST mission {k + 1}', good,
             f'{extra}state {info and info["state"]}, pallet {pad_cm:.1f} cm from the pad centre')
        if rate is not None:
            oks.append(rate >= 1.8)
            line('event stream rate (>= 1.8 Hz)', rate >= 1.8, f'{rate:.2f} Hz')
    return all(oks)


def case_cancel(rig):
    """Cancel over REST ends the mission as CANCELED and stops the robot."""
    rig.reset_world()
    mission_id = call('POST', '/missions', {**GOOD, 'mission_id': f'web-c{int(time.time())}'}
                      ).json()['mission_id']
    time.sleep(15)
    resp = call('POST', f'/missions/{mission_id}/cancel')
    info = wait_mission(mission_id, 120)
    stopped = rig.stopped()
    orphans = rig.orphan_goals()
    ok = (resp.status_code == 200 and info is not None and info['error_code'] == 'CANCELED'
          and stopped and not orphans)
    return line('cancel over REST', ok, f'{resp.status_code}, {info and info["error_code"]}, '
                f'stopped {stopped}, orphans {orphans or "-"}')


def case_ros_down(rig):
    """With the mission manager stopped, POST /missions gives 503 and /health says ros false."""
    subprocess.run([STACK, 'stop_mission'], capture_output=True)
    time.sleep(2)
    resp = call('POST', '/missions', GOOD)
    health = call('GET', '/health', headers=None).json()
    ok = (resp.status_code == 503 and resp.json()['error_code'] == 'ROS_UNAVAILABLE'
          and health['ros'] is False and health['status'] == 'ok')
    line('mission manager down -> 503', ok, f'{resp.status_code}, health {health}')
    subprocess.run([STACK, 'start_mission'], capture_output=True)
    time.sleep(3)
    back = call('GET', '/health', headers=None).json()['ros']
    return line('mission manager back -> ros true', back and ok, str(back))


def rosbridge_attempts(topics):
    """Try to publish Twists through rosbridge to each topic; return when done."""
    async def run():
        async with websockets.connect(WS) as ws:
            for topic in topics:
                await ws.send(json.dumps({'op': 'advertise', 'topic': topic,
                                          'type': 'geometry_msgs/Twist'}))
            end = time.time() + 2.0
            while time.time() < end:
                for topic in topics:
                    await ws.send(json.dumps({'op': 'publish', 'topic': topic,
                                              'msg': {'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0},
                                                      'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}}))
                await asyncio.sleep(0.05)

    asyncio.run(run())


def rosbridge_service(service, service_type):
    """Call a service through rosbridge; return the raw reply (or None on timeout)."""
    async def run():
        async with websockets.connect(WS) as ws:
            await ws.send(json.dumps({'op': 'call_service', 'service': service,
                                      'type': service_type, 'args': {}, 'id': 'x1'}))
            try:
                return json.loads(await asyncio.wait_for(ws.recv(), 5))
            except asyncio.TimeoutError:
                return None

    return asyncio.run(run())


def case_estop(rig):
    """E-stop: stops within 1.5 s, cancels the mission, blocks teleop, no remote release."""
    rig.reset_world()
    mission_id = call('POST', '/missions', {**GOOD, 'mission_id': f'web-e{int(time.time())}'}
                      ).json()['mission_id']
    t0 = time.time()
    while time.time() - t0 < 60 and math.hypot(rig.gt.twist.twist.linear.x,
                                               rig.gt.twist.twist.linear.y) < 0.05:
        rig.spin(0.2)
    moving = math.hypot(rig.gt.twist.twist.linear.x, rig.gt.twist.twist.linear.y) >= 0.05
    t_stop = time.time()
    resp = call('POST', '/estop')
    stop_time = None
    while time.time() - t_stop < 5:
        rig.spin(0.05)
        if math.hypot(rig.gt.twist.twist.linear.x, rig.gt.twist.twist.linear.y) < 0.02:
            stop_time = time.time() - t_stop
            break
    info = wait_mission(mission_id, 60)
    stopped = rig.stopped(3.0)
    results = [line('e-stop engages and stops the robot', moving and resp.status_code == 200
                    and stop_time is not None and stop_time <= 1.5 and stopped,
                    f'moving before {moving}, stopped after '
                    f'{stop_time and round(stop_time, 2)} s')]
    results.append(line('e-stop cancels the mission', info is not None
                        and info['error_code'] == 'CANCELED' and not rig.orphan_goals(),
                        f'{info and info["error_code"]}, orphans {rig.orphan_goals() or "-"}'))
    new = call('POST', '/missions', GOOD)
    results.append(line('mission refused while engaged', new.status_code == 423
                        and new.json()['error_code'] == 'ESTOP_ENGAGED', str(new.status_code)))
    release = call('POST', '/estop/release')
    results.append(line('REST cannot release the e-stop', release.status_code == 403
                        and call('GET', '/status').json()['estop'] is True,
                        f'{release.status_code} {release.json().get("error_code")}'))
    reply = rosbridge_service('/estop/release', 'std_srvs/Trigger')
    still = call('GET', '/status').json()['estop'] is True
    results.append(line('rosbridge cannot release the e-stop', still,
                        f'reply {str(reply)[:90]}'))
    teleop = rig.create_publisher(Twist, '/cmd_vel_teleop', 10)
    x0, y0, _ = rig.pose()
    t1 = time.time()
    while time.time() - t1 < 2.0:
        msg = Twist()
        msg.linear.x = 0.2
        teleop.publish(msg)
        rig.spin(0.05)
    x1, y1, _ = rig.pose()
    results.append(line('teleop cannot override the e-stop', math.hypot(x1 - x0, y1 - y0) < 0.005,
                        f'moved {math.hypot(x1 - x0, y1 - y0) * 1000:.1f} mm'))
    local = subprocess.run(['ros2', 'service', 'call', '/estop/release', 'std_srvs/srv/Trigger'],
                           capture_output=True, text=True).stdout
    rig.spin(2.0)
    released = call('GET', '/status').json()['estop'] is False
    again = call('POST', '/missions', {**GOOD, 'mission_id': f'web-r{int(time.time())}'})
    results.append(line('local release re-enables missions', released and 'success=True' in local
                        and again.status_code == 202,
                        f'released {released}, POST {again.status_code}'))
    if again.status_code == 202:
        call('POST', f'/missions/{again.json()["mission_id"]}/cancel')
        wait_mission(again.json()['mission_id'], 120)
    return all(results)


def case_rosbridge(rig):
    """Check the allow-list: telemetry readable, no publish reaches the velocity inputs."""
    seen = {'cmd_vel': 0, 'cmd_vel_teleop': 0, 'cmd_vel_dock': 0, 'cmd_vel_estop': 0}
    subs = [rig.create_subscription(Twist, f'/{name}',
                                    lambda m, n=name: seen.__setitem__(n, seen[n] + 1), 10)
            for name in seen]
    # the gate and the mux publish on /cmd_vel; count only what appears while the robot is idle
    base_vel = seen['cmd_vel']
    rig.spin(1.0)
    idle_rate = seen['cmd_vel'] - base_vel
    for name in ('cmd_vel_teleop', 'cmd_vel_dock', 'cmd_vel_estop'):
        seen[name] = 0

    async def telemetry():
        got = {}
        async with websockets.connect(WS) as ws:
            for topic in ('/lift_state', '/mission_status', '/odom'):
                await ws.send(json.dumps({'op': 'subscribe', 'topic': topic}))
            end = time.time() + 4
            while time.time() < end:
                try:
                    msg = json.loads(await asyncio.wait_for(ws.recv(), 1))
                except asyncio.TimeoutError:
                    continue
                if msg.get('op') == 'publish':
                    got[msg['topic']] = got.get(msg['topic'], 0) + 1
        return got

    got = asyncio.run(telemetry())
    results = [line('rosbridge: allowed telemetry readable', got.get('/lift_state', 0) > 0
                    and got.get('/mission_status', 0) > 0, str(got)),
               line('rosbridge: /odom not in the allow-list', got.get('/odom', 0) == 0, '')]
    x0, y0, _ = rig.pose()
    rosbridge_attempts(['/cmd_vel_teleop', '/cmd_vel_dock', '/cmd_vel_estop', '/cmd_vel'])
    rig.spin(1.0)
    x1, y1, _ = rig.pose()
    leaked = {k: v for k, v in seen.items() if k != 'cmd_vel' and v}
    results.append(line('rosbridge: no publish to velocity topics', not leaked
                        and math.hypot(x1 - x0, y1 - y0) < 0.005,
                        f'delivered {leaked or "none"}, moved '
                        f'{math.hypot(x1 - x0, y1 - y0) * 1000:.1f} mm, /cmd_vel idle msgs '
                        f'{idle_rate}'))
    reply = rosbridge_service('/execute_mission/_action/cancel_goal', 'action_msgs/CancelGoal')
    results.append(line('rosbridge: services refused', reply is None or reply.get('result')
                        is False, str(reply)[:100]))
    for sub in subs:
        rig.destroy_subscription(sub)
    return all(results)


def case_video(rig):
    """Check that the MJPEG stream carries JPEG frames and the dashboard page loads."""
    got = b''
    ctype = ''
    with httpx.stream('GET', VIDEO, timeout=20) as resp:
        ctype = resp.headers.get('content-type', '')
        for chunk in resp.iter_bytes():
            got += chunk
            if len(got) > 150000:
                break
    page = httpx.get(BASE + '/', timeout=10)
    return line('video stream + dashboard', 'multipart' in ctype and b'\xff\xd8' in got
                and page.status_code == 200 and 'E-STOP' in page.text,
                f'{ctype[:40]}, {len(got)} bytes, dashboard {page.status_code}')


def case_estop_without_mission_layer(rig):
    """Check the e-stop with the mission manager stopped, against a robot driven by teleop."""
    rig.reset_world()
    subprocess.run([STACK, 'stop_mission'], capture_output=True)
    time.sleep(2)
    teleop = rig.create_publisher(Twist, '/cmd_vel_teleop', 10)

    def drive(seconds):
        t1 = time.time()
        while time.time() - t1 < seconds:
            msg = Twist()
            msg.linear.y = 0.2
            teleop.publish(msg)
            rig.spin(0.05)

    drive(2.0)
    speed = math.hypot(rig.gt.twist.twist.linear.x, rig.gt.twist.twist.linear.y)
    resp = call('POST', '/estop')
    t0 = time.time()
    stop_time = None
    while time.time() - t0 < 5:                       # teleop keeps commanding the whole time
        drive(0.05)
        if math.hypot(rig.gt.twist.twist.linear.x, rig.gt.twist.twist.linear.y) < 0.02:
            stop_time = time.time() - t0
            break
    x0, y0, _ = rig.pose()
    drive(2.0)
    x1, y1, _ = rig.pose()
    held = math.hypot(x1 - x0, y1 - y0) < 0.005
    subprocess.run(['ros2', 'service', 'call', '/estop/release', 'std_srvs/srv/Trigger'],
                   capture_output=True)
    subprocess.run([STACK, 'start_mission'], capture_output=True)
    return line('e-stop with the mission layer down', speed > 0.1 and resp.status_code == 200
                and stop_time is not None and stop_time <= 1.5 and held,
                f'speed before {speed:.2f} m/s, stopped after '
                f'{stop_time and round(stop_time, 2)} s, held against teleop {held}')


CASES = {1: case_listeners, 2: case_auth_and_invalid, 3: case_missions, 4: case_cancel,
         5: case_ros_down, 6: case_estop, 7: case_rosbridge, 8: case_video,
         9: case_estop_without_mission_layer}


def main():
    """Run the selected cases."""
    wanted = [int(c) for c in sys.argv[1].split(',')] if len(sys.argv) > 1 else list(CASES)
    rclpy.init()
    rig = Rig()
    while rig.gt is None:
        rclpy.spin_once(rig, timeout_sec=0.1)
    results = {n: CASES[n](rig) for n in wanted}
    print(f'passed {sum(results.values())}/{len(results)}: '
          f'{ {k: ("PASS" if v else "FAIL") for k, v in results.items()} }')
    rig.reset_world(navigate=False)


if __name__ == '__main__':
    main()
