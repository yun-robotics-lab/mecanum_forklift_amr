"""ROS side of the REST API: mission client, status aggregation, e-stop access (no HTTP here)."""
from collections import OrderedDict
import math
import threading
import time

from geometry_msgs.msg import Twist
from mecanum_forklift_interfaces.action import ExecuteMission
from mecanum_forklift_interfaces.msg import LiftState, MissionStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from mecanum_forklift_web_bridge.validators import ApiError

LIFT_STATES = {0: 'UNINITIALIZED', 1: 'HOMING', 2: 'IDLE', 3: 'MOVING', 4: 'FAULT'}
KEEP_MISSIONS = 100


class RosBackend(Node):
    """Backend of api.create_app for the real robot / simulation."""

    def __init__(self):
        """Declare parameters, load the dock database, create subscriptions and clients."""
        super().__init__('web_backend')
        for name, default in (('dock_database', ''), ('video_url', ''),
                              ('mission_action', 'execute_mission'),
                              ('estop_velocity_topic', 'cmd_vel_estop')):
            self.declare_parameter(name, default)
        p = self.get_parameter
        self.video_url = p('video_url').value
        self.docks_all = {}
        if p('dock_database').value:
            with open(p('dock_database').value, encoding='utf-8') as f:
                self.docks_all = (yaml.safe_load(f) or {}).get('docks', {}) or {}
        self.lock = threading.Lock()
        self.missions = OrderedDict()
        self.handles = {}
        self.mission_status = None
        self.lift = None
        self.estop = None
        self.fallback_until = 0.0
        group = ReentrantCallbackGroup()
        self.create_subscription(MissionStatus, 'mission_status', self.on_mission_status, 10,
                                 callback_group=group)
        self.create_subscription(LiftState, 'lift_state', self.on_lift, 10,
                                 callback_group=group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'estop/state', self.on_estop, latched,
                                 callback_group=group)
        self.mission_client = ActionClient(self, ExecuteMission, p('mission_action').value,
                                           callback_group=group)
        self.estop_client = self.create_client(Trigger, 'estop/engage', callback_group=group)
        self.fallback_pub = self.create_publisher(Twist, p('estop_velocity_topic').value, 10)
        self.create_timer(0.05, self.fallback_tick, callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    # ---- subscriptions ----------------------------------------------------------------
    def on_mission_status(self, msg):
        """Keep the latest mission manager status."""
        self.mission_status = msg

    def on_lift(self, msg):
        """Keep the latest lift state."""
        self.lift = msg

    def on_estop(self, msg):
        """Keep the latched e-stop state."""
        self.estop = bool(msg.data)

    # ---- backend interface used by the API ---------------------------------------------
    def connected(self):
        """Return True while the mission manager's action server is available."""
        return self.mission_client.server_is_ready()

    def docks(self):
        """Public dock view: name -> operation and type (test-only entries hidden)."""
        return {name: {'operation': e.get('operation', 'pick'), 'type': e.get('type', '')}
                for name, e in self.docks_all.items() if not e.get('test_only')}

    def estop_engaged(self):
        """Return True if the e-stop is engaged or its state is unknown (fail safe)."""
        return self.estop is not False

    def lift_fault(self):
        """Return True if the lift reports a fault."""
        return self.lift is not None and self.lift.state == LiftState.STATE_FAULT

    def mission_active(self):
        """Return True if the manager or an earlier request says a mission is running."""
        with self.lock:
            own = any(m['active'] for m in self.missions.values())
        return own or bool(self.mission_status is not None and self.mission_status.active)

    def has_mission(self, mission_id):
        """Return True if the id was used by a mission submitted through this API."""
        with self.lock:
            return mission_id in self.missions

    def mission(self, mission_id):
        """Return a copy of the mission record or None."""
        with self.lock:
            record = self.missions.get(mission_id)
            return dict(record) if record else None

    def submit(self, clean):
        """Send the ExecuteMission goal (non-blocking); results update the record."""
        if not self.mission_client.server_is_ready():
            raise ApiError(503, 'ROS_UNAVAILABLE', 'the mission manager is not available')
        goal = ExecuteMission.Goal()
        goal.mission_id = clean['mission_id']
        goal.mission_type = clean['mission_type']
        goal.pick_dock_id, goal.drop_dock_id = clean['pick_dock_id'], clean['drop_dock_id']
        pose = clean['return_pose']
        if pose is not None:
            goal.return_pose.header.frame_id = 'map'
            position = goal.return_pose.pose.position
            position.x, position.y = pose['x'], pose['y']
            goal.return_pose.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
            goal.return_pose.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
        mission_id = clean['mission_id']
        record = {'mission_id': mission_id, 'active': True, 'state': 'SUBMITTED',
                  'step_index': 0, 'total_steps': 0, 'success': None, 'error_code': '',
                  'failed_step': '', 'message': '', 'submitted_at': time.time()}
        with self.lock:
            self.missions[mission_id] = record
            while len(self.missions) > KEEP_MISSIONS:
                self.missions.popitem(last=False)

        def feedback(msg):
            fb = msg.feedback
            self.update(mission_id, state=fb.state, step_index=fb.step_index,
                        total_steps=fb.total_steps, message=fb.message)

        def accepted(future):
            handle = future.result()
            if not handle.accepted:
                self.update(mission_id, active=False, state='REJECTED', success=False,
                            error_code='REJECTED', message='the mission manager rejected the goal')
                return
            with self.lock:
                self.handles[mission_id] = handle
            handle.get_result_async().add_done_callback(finished)

        def finished(future):
            result = future.result().result
            self.update(mission_id, active=False, success=bool(result.success),
                        error_code=result.error_code, failed_step=result.failed_step,
                        message=result.message,
                        state='DONE' if result.success else (
                            'CANCELED' if result.error_code == 'CANCELED' else 'FAILED'))
            with self.lock:
                self.handles.pop(mission_id, None)

        self.mission_client.send_goal_async(goal, feedback_callback=feedback) \
            .add_done_callback(accepted)

    def update(self, mission_id, **fields):
        """Update a mission record."""
        with self.lock:
            if mission_id in self.missions:
                self.missions[mission_id].update(fields)

    def cancel(self, mission_id):
        """Ask the mission manager to cancel the mission."""
        with self.lock:
            handle = self.handles.get(mission_id)
        if handle is None:
            raise ApiError(409, 'NOT_ACTIVE', 'the mission has no active goal yet')
        handle.cancel_goal_async()

    def engage_estop(self):
        """Call the e-stop node; if it is unreachable, hold zero velocity here as a fallback."""
        if self.estop_client.wait_for_service(timeout_sec=0.5):
            future = self.estop_client.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not future.done() and time.monotonic() - t0 < 1.5:
                time.sleep(0.01)
            if future.done() and future.result().success:
                return {'estop': 'engaged', 'via': 'estop_node'}
        self.fallback_until = time.monotonic() + 5.0
        self.get_logger().error('estop_node unreachable: fallback zero velocity for 5 s')
        return {'estop': 'engaged', 'via': 'fallback', 'warning': 'estop_node unreachable; '
                'zero velocity held for 5 s only, goals NOT cancelled'}

    def fallback_tick(self):
        """Publish zero velocity while the fallback stop is active."""
        if time.monotonic() < self.fallback_until:
            self.fallback_pub.publish(Twist())

    def status(self):
        """Aggregate the status JSON."""
        pose = None
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
            q = t.rotation
            pose = {'frame': 'map', 'x': round(t.translation.x, 3),
                    'y': round(t.translation.y, 3),
                    'yaw': round(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                            1 - 2 * (q.y * q.y + q.z * q.z)), 3)}
        except TransformException:
            pass
        lift = None
        if self.lift is not None:
            lift = {'state': LIFT_STATES.get(self.lift.state, str(self.lift.state)),
                    'fault_code': int(self.lift.fault_code), 'height': round(self.lift.height, 4),
                    'target_height': round(self.lift.target_height, 4)}
        ms = self.mission_status
        mission = {'active': False, 'mission_id': '', 'state': 'IDLE', 'step_index': 0,
                   'total_steps': 0, 'attempt': 0, 'error_code': '', 'message': ''}
        if ms is not None:
            mission = {'active': bool(ms.active), 'mission_id': ms.mission_id, 'state': ms.state,
                       'step_index': ms.step_index, 'total_steps': ms.total_steps,
                       'attempt': ms.attempt, 'error_code': ms.error_code, 'message': ms.message}
        return {'time': time.time(), 'estop': self.estop_engaged(),
                'estop_known': self.estop is not None, 'pose': pose, 'lift': lift,
                'mission': mission, 'mission_manager_available': self.connected(),
                'battery': None, 'diagnostics': None, 'video_url': self.video_url}
