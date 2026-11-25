#!/usr/bin/env python3
"""
DockPallet action server (marker, coarse approach, final alignment, lift, back out).

Sequence: marker -> opennav_docking approach -> holonomic final alignment -> fork lift -> back out.

Structure adapted from the reference `PalletDockWorkflowNode` (dock -> lift -> undock, cancelable,
state feedback, error codes). Differences: the target marker id and lift height come from the
dock database, the marker node is set before docking, a total timeout applies, and every
sub-action failure maps to a distinct error code. Velocity goes out only on cmd_vel_dock
(docking_server during the approach, this node during the final alignment), always through
twist_mux and cmd_vel_gate; zero velocity is sent whenever the alignment ends.
"""
import math
import os
import time

from geometry_msgs.msg import Point32, Polygon, Twist
from mecanum_forklift_docking.align_core import (
    align_step, AlignParams, robot_frame_errors, target_from_marker, within_tolerance)
from mecanum_forklift_docking.dock_core import (
    dock_error_name, load_docks, lookup_dock, polygon_from_flat, progress_of, staging_pose,
    to_frame, wrong_pallet)
from mecanum_forklift_interfaces.action import DockPallet, SetLiftHeight
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from opennav_docking_msgs.action import DockRobot, UndockRobot
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Failure(Exception):
    """A step failed; carries the DockPallet error_code and a message."""

    def __init__(self, code, message):
        """Store the error code and message."""
        super().__init__(message)
        self.code = code
        self.message = message


class PalletDockWorkflow(Node):
    """DockPallet action server."""

    def __init__(self):
        """Declare parameters, load the database, create clients and the action server."""
        super().__init__('pallet_dock_workflow')
        self.declare_parameter('dock_database', '')
        self.declare_parameter('total_timeout', 90.0)
        self.declare_parameter('lift_speed', 0.0)
        self.declare_parameter('marker_node', 'aruco_node')
        self.declare_parameter('perception_timeout', 20.0)  # wall s to see a fresh marker pose
        self.declare_parameter('staging_x_offset', -0.25)   # same as docking_server.yaml
        self.declare_parameter('nav_map_default', '')        # curated Nav2 map (full yaml path)
        self.declare_parameter('nav_map_service', 'nav_map_server/load_map')
        self.declare_parameter('stop_distance', 0.245)      # m, base_link before the marker
        self.declare_parameter('latch_distance', 0.5)       # m, marker poses closer are ignored
        self.declare_parameter('align_timeout', 30.0)       # s
        self.declare_parameter('align_dwell', 0.5)          # s inside tolerance before success
        self.declare_parameter('tol_long', 0.010)
        self.declare_parameter('tol_lat', 0.010)
        self.declare_parameter('tol_yaw_deg', 2.0)
        # footprints published to the Nav2 costmaps (x, y pairs): without / with the pallet
        self.declare_parameter('unloaded_footprint', [0.368, 0.154, 0.368, -0.154,
                                                      -0.205, -0.154, -0.205, 0.154])
        self.declare_parameter('loaded_footprint', [0.435, 0.154, 0.435, -0.154,
                                                    -0.205, -0.154, -0.205, 0.154])
        self.declare_parameter('footprint_topics',
                               ['local_costmap/footprint', 'global_costmap/footprint'])
        self.declare_parameter('align_max_vx', 0.08)
        self.declare_parameter('align_max_vy', 0.08)
        self.declare_parameter('align_max_wz', 0.25)
        p = self.get_parameter
        self.docks = load_docks(p('dock_database').value)
        self.perception_timeout = p('perception_timeout').value
        self.last_pose_rx = None
        self.latched = None        # (x, y, yaw) of the marker in odom, from the last far sighting
        self.stop_distance = p('stop_distance').value
        self.latch_distance = p('latch_distance').value
        self.align_timeout = p('align_timeout').value
        self.align_dwell = p('align_dwell').value
        self.align = AlignParams(
            tol_long=p('tol_long').value, tol_lat=p('tol_lat').value,
            tol_yaw=math.radians(p('tol_yaw_deg').value), max_vx=p('align_max_vx').value,
            max_vy=p('align_max_vy').value, max_wz=p('align_max_wz').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_dock', 10)
        self.unloaded = polygon_from_flat(p('unloaded_footprint').value)
        self.loaded = polygon_from_flat(p('loaded_footprint').value)
        self.footprint_pubs = [self.create_publisher(Polygon, t, 10)
                               for t in p('footprint_topics').value]
        self.carrying = False
        self.create_timer(1.0, self.on_footprint_timer)
        self.create_subscription(PoseStamped, 'aruco_dock_pose', self.on_pose, 10)
        self.lift_speed = p('lift_speed').value
        group = ReentrantCallbackGroup()
        self.dock_client = ActionClient(self, DockRobot, 'dock_robot', callback_group=group)
        self.undock_client = ActionClient(self, UndockRobot, 'undock_robot',
                                          callback_group=group)
        self.lift_client = ActionClient(self, SetLiftHeight, 'set_lift_height',
                                        callback_group=group)
        self.map_client = self.create_client(LoadMap, p('nav_map_service').value,
                                             callback_group=group)
        self.map_swapped = False
        self.carried_pallet = None     # `pallet` of the last pick entry, if it names one
        self.fork_at_rack_height = False
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.param_client = self.create_client(
            SetParameters, f'{p("marker_node").value}/set_parameters', callback_group=group)
        self.server = ActionServer(
            self, DockPallet, 'dock_pallet', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.current = None        # active sub-goal handle, for cancel propagation
        self.busy = False
        self.get_logger().info(f'dock_pallet ready, docks: {list(self.docks)}')

    def load_nav_map(self, yaml_path):
        """Ask the Nav2 map server for another curated map; return True on success."""
        if not yaml_path or not self.map_client.wait_for_service(timeout_sec=3.0):
            return False
        future = self.map_client.call_async(LoadMap.Request(map_url=yaml_path))
        t0 = time.time()
        while not future.done() and time.time() - t0 < 10.0:
            time.sleep(0.05)
        ok = future.done() and future.result().result == LoadMap.Response.RESULT_SUCCESS
        if not ok:
            self.get_logger().error(f'could not load map {yaml_path}')
        return ok

    def publish_footprint(self, points):
        """Publish a footprint polygon to the costmaps."""
        msg = Polygon()
        msg.points = [Point32(x=x, y=y, z=0.0) for x, y in points]
        for pub in self.footprint_pubs:
            pub.publish(msg)

    def set_carrying(self, carrying):
        """Switch the costmap footprint between the bare fork and the fork with the pallet."""
        self.carrying = carrying
        for _ in range(3):
            self.publish_footprint(self.loaded if carrying else self.unloaded)
            time.sleep(0.1)
        self.get_logger().info(f'costmap footprint: {"loaded" if carrying else "unloaded"}')

    def on_footprint_timer(self):
        """Keep re-publishing the loaded footprint while carrying (costmaps may restart)."""
        if self.carrying:
            self.publish_footprint(self.loaded)

    def robot_pose(self, frame='odom'):
        """base_link (x, y, yaw) in `frame` from TF, or None."""
        try:
            t = self.tf_buffer.lookup_transform(frame, 'base_link', Time()).transform
        except TransformException:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def on_pose(self, msg):
        """Latch the marker pose while it is far enough for a reliable sighting."""
        self.last_pose_rx = time.monotonic()
        robot = self.robot_pose()
        if robot is None:
            return
        o = msg.pose.orientation
        psi = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        mx, my = msg.pose.position.x, msg.pose.position.y
        if math.hypot(mx - robot[0], my - robot[1]) > self.latch_distance:
            self.latched = (mx, my, psi)

    def on_goal(self, goal):
        """Accept one goal at a time."""
        if self.busy:
            self.get_logger().warn('rejecting dock_pallet goal: already docking')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ---- helpers -------------------------------------------------------------------------
    def wait(self, future, goal_handle, deadline):
        """Block until a future completes; raise on cancel or timeout."""
        while not future.done():
            if goal_handle.is_cancel_requested:
                self.cancel_current()
                raise Failure('CANCELED', 'canceled')
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                self.cancel_current()
                raise Failure('TIMEOUT', 'total timeout')
            time.sleep(0.05)
        return future.result()

    def cancel_current(self):
        """Cancel the active sub-action (the docking server stops the robot)."""
        if self.current is not None:
            self.current.cancel_goal_async()

    def run_action(self, client, goal, goal_handle, deadline, name, full=False):
        """Send a sub-goal and wait for its result (full=True: the response with .status)."""
        if not client.wait_for_server(timeout_sec=3.0):
            raise Failure('SERVER_UNAVAILABLE', f'{name} action server not available')
        handle = self.wait(client.send_goal_async(goal), goal_handle, deadline)
        if not handle.accepted:
            raise Failure('DOCK_FAILED', f'{name} goal rejected')
        self.current = handle
        try:
            response = self.wait(handle.get_result_async(), goal_handle, deadline)
            return response if full else response.result
        finally:
            self.current = None

    def feedback(self, goal_handle, state):
        """Publish state feedback."""
        fb = DockPallet.Feedback()
        fb.state = state
        fb.progress = progress_of(state)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'state {state}')

    # ---- main ----------------------------------------------------------------------------
    def execute(self, goal_handle):
        """Run the sequence for one DockPallet goal."""
        self.busy = True
        req = goal_handle.request
        result = DockPallet.Result()
        timeout = self.get_parameter('total_timeout').value      # live-settable
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        try:
            self.sequence(goal_handle, req, deadline)
            self.feedback(goal_handle, 'DONE')
            result.success = True
            result.message = f'docked at {req.dock_id}'
            goal_handle.succeed()
        except Failure as exc:
            self.get_logger().error(f'{exc.code}: {exc.message}')
            result.success, result.error_code, result.message = False, exc.code, exc.message
            if exc.code == 'CANCELED':
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self.busy = False
            self.current = None
        return result

    def set_marker_param(self, marker_id, goal_handle, deadline, strict=True):
        """Tell the vision node which marker to publish; raise MARKER_SET_FAILED if strict."""
        if not self.param_client.wait_for_service(timeout_sec=3.0):
            if strict:
                raise Failure('MARKER_SET_FAILED', 'marker node unavailable')
            return
        prm = Parameter(name='target_marker_id', value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=marker_id))
        resp = self.wait(self.param_client.call_async(SetParameters.Request(parameters=[prm])),
                         goal_handle, deadline)
        if strict and (not resp.results or not resp.results[0].successful):
            raise Failure('MARKER_SET_FAILED', 'could not set target_marker_id')

    def send_velocity(self, vx, vy, wz):
        """Publish one body-velocity command on cmd_vel_dock."""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.vel_pub.publish(msg)

    def stop(self):
        """Command zero velocity a few times (the gate also times out to zero)."""
        for _ in range(5):
            self.send_velocity(0.0, 0.0, 0.0)
            time.sleep(0.02)

    def back_off(self, goal_handle, deadline, distance, speed=0.08):
        """Reverse `distance` metres by dead reckoning on odom (out from a dropped pallet)."""
        start = self.robot_pose()
        if start is None:
            raise Failure('UNDOCK_FAILED', 'no odom -> base_link transform')
        t0 = self.get_clock().now().nanoseconds * 1e-9
        try:
            while True:
                now = self.get_clock().now().nanoseconds * 1e-9
                if goal_handle.is_cancel_requested:
                    raise Failure('CANCELED', 'canceled')
                if now > deadline:
                    raise Failure('TIMEOUT', 'total timeout')
                pose = self.robot_pose()
                if pose is not None and math.hypot(pose[0] - start[0],
                                                   pose[1] - start[1]) >= distance:
                    return
                if now - t0 > distance / speed * 4 + 5.0:
                    raise Failure('UNDOCK_FAILED', 'back-off did not finish')
                self.send_velocity(-speed, 0.0, 0.0)
                time.sleep(0.05)
        finally:
            self.stop()

    def final_align(self, goal_handle, deadline, target, frame='odom'):
        """Holonomic alignment on `target` (x, y, yaw in `frame`) until inside the tolerances."""
        start = self.get_clock().now().nanoseconds * 1e-9
        inside_since = None
        try:
            while True:
                now = self.get_clock().now().nanoseconds * 1e-9
                if goal_handle.is_cancel_requested:
                    raise Failure('CANCELED', 'canceled')
                if now > deadline:
                    raise Failure('TIMEOUT', 'total timeout')
                robot = self.robot_pose(frame)
                if robot is None:
                    raise Failure('ALIGN_FAILED', f'no {frame} -> base_link transform')
                ex, ey, eyaw = robot_frame_errors(robot, target)
                if within_tolerance(ex, ey, eyaw, self.align):
                    inside_since = inside_since if inside_since is not None else now
                    self.send_velocity(0.0, 0.0, 0.0)
                    if now - inside_since >= self.align_dwell:
                        self.get_logger().info(
                            f'aligned: long {ex * 1000:.1f} mm, lat {ey * 1000:.1f} mm, '
                            f'yaw {math.degrees(eyaw):.2f} deg')
                        return
                else:
                    inside_since = None
                    self.send_velocity(*align_step(ex, ey, eyaw, self.align))
                if now - start > self.align_timeout:
                    raise Failure('ALIGN_FAILED', f'alignment timeout: long {ex * 1000:.1f} mm, '
                                  f'lat {ey * 1000:.1f} mm, yaw {math.degrees(eyaw):.2f} deg')
                time.sleep(0.05)
        finally:
            self.stop()

    def navigate_to(self, goal_handle, deadline, frame, x, y, yaw):
        """Navigate with Nav2; raise STAGING_FAILED unless it succeeds."""
        nav = NavigateToPose.Goal()
        nav.pose.header.frame_id = frame
        nav.pose.pose.position.x, nav.pose.pose.position.y = x, y
        nav.pose.pose.orientation.z = math.sin(yaw / 2.0)
        nav.pose.pose.orientation.w = math.cos(yaw / 2.0)
        res = self.run_action(self.nav_client, nav, goal_handle, deadline, 'navigate_to_pose',
                              full=True)
        if res.status != 4:                           # not STATUS_SUCCEEDED
            raise Failure('STAGING_FAILED',
                          f'navigation to the staging pose ended with status {res.status}')

    def set_fork(self, goal_handle, deadline, height):
        """Move the fork through the lift action; raise LIFT_FAILED on any failure."""
        lift_goal = SetLiftHeight.Goal()
        lift_goal.target_height = float(height)
        lift_goal.speed = float(self.lift_speed)
        lres = self.run_action(self.lift_client, lift_goal, goal_handle, deadline,
                               'set_lift_height')
        if not lres.success:
            raise Failure('LIFT_FAILED', f'lift failed (code {lres.error_code}): {lres.message}')

    def raise_for_rack(self, goal_handle, deadline, height):
        """Raise the fork above a rack deck before driving up to it."""
        self.fork_at_rack_height = True
        self.set_fork(goal_handle, deadline, height)

    def freeze_in_odom(self, target, frame):
        """
        Fix a drop target of `frame` in odom, using the localization of this moment.

        Needed for rack drops: the raised load enters the lidar plane and would disturb AMCL
        during the approach.
        """
        robot = self.robot_pose(frame)
        robot_odom = self.robot_pose('odom')
        if robot is None or robot_odom is None:
            raise Failure('ALIGN_FAILED', f'no transform from {frame} or odom to base_link')
        return to_frame(target, robot, robot_odom)

    def swap_map(self, entry, key):
        """Load the curated map named by entry[key] (next to the default map), if configured."""
        after = entry.get(key)
        default = self.get_parameter('nav_map_default').value
        if after and default:
            self.map_swapped = self.load_nav_map(os.path.join(os.path.dirname(default), after))
            if not self.map_swapped:
                raise Failure('MAP_UPDATE_FAILED', f'could not load the map of {key}')

    def sequence(self, goal_handle, req, deadline):
        """Run the pick or the drop of the dock entry."""
        entry = self.docks.get(req.dock_id)
        if entry is None:
            raise Failure('DOCK_NOT_IN_DB', f'unknown dock {req.dock_id}')
        if entry.get('operation', 'pick') == 'drop':
            self.drop_sequence(goal_handle, req, deadline, entry)
        else:
            self.pick_sequence(goal_handle, req, deadline, entry)

    def drop_sequence(self, goal_handle, req, deadline, entry):
        """
        STAGING -> ALIGNING -> LOWERING -> UNDOCKING, without a marker.

        A rack drop entry has `approach_lift`: the fork (with the pallet) is raised at the staging
        pose, lowered to `lift` inside the rack, and set to `fork_after` once backed out.

        With a pallet on the forks the fork-mounted camera looks into the load, so the drop uses
        poses of the `map` frame from the dock entry (staging_pose, final_pose; AMCL accuracy).
        """
        if not self.carrying:
            raise Failure('NOT_CARRYING', 'a drop needs a pallet on the forks')
        if wrong_pallet(entry, self.carried_pallet):
            raise Failure('WRONG_PALLET', f'this drop is for {entry["pallet"]}, the forks carry '
                          f'{self.carried_pallet} (its map would be wrong)')
        frame = entry.get('frame', 'map')
        self.feedback(goal_handle, 'SET_MARKER')
        if req.navigate_to_staging:
            self.feedback(goal_handle, 'STAGING')
            sx, sy, syaw = (float(v) for v in entry['staging_pose'])
            self.navigate_to(goal_handle, deadline, frame, sx, sy, syaw)
        target = tuple(float(v) for v in entry['final_pose'])
        if 'approach_lift' in entry:
            target, frame = self.freeze_in_odom(target, frame), 'odom'
            self.raise_for_rack(goal_handle, deadline, entry['approach_lift'])
        self.feedback(goal_handle, 'ALIGNING')
        self.final_align(goal_handle, deadline, target, frame)
        self.feedback(goal_handle, 'LOWERING')
        self.set_fork(goal_handle, deadline, entry.get('lift', 0.0))
        self.set_carrying(False)                      # the pallet now stands on the floor / rack
        self.carried_pallet = None
        self.swap_map(entry, 'map_after_drop')        # floor: a painted obstacle from now on
        if req.undock_after:
            self.feedback(goal_handle, 'UNDOCKING')
            self.back_off(goal_handle, deadline, float(entry.get('backoff', 0.30)))
            if 'fork_after' in entry:                 # only once the tines are out
                self.set_fork(goal_handle, deadline, entry['fork_after'])
                self.fork_at_rack_height = False

    def pick_sequence(self, goal_handle, req, deadline, entry):
        """
        SET_MARKER -> (STAGING) -> DOCKING -> ALIGNING -> LIFTING -> (UNDOCKING).

        A rack pick entry has `approach_lift` (fork height to enter the pallet above the deck),
        `backoff` (straight reverse instead of undock_robot) and `carry_lift` (set after it).
        """
        info = lookup_dock(self.docks, req.dock_id)
        if info is None:
            raise Failure('DOCK_NOT_IN_DB', f'dock {req.dock_id} has no marker_id')
        marker_id, lift = info
        stop_distance = float(entry.get('stop_distance', self.stop_distance))

        self.feedback(goal_handle, 'SET_MARKER')
        self.latched = None
        self.set_carrying(False)              # a pick starts with the bare fork
        # a floor pick means its pallet is back in its slot: restore the map. A rack pick changes
        # no map and must keep the current one.
        if self.map_swapped and entry.get('map_after_pick'):
            self.map_swapped = not self.load_nav_map(self.get_parameter('nav_map_default').value)
        self.set_marker_param(marker_id, goal_handle, deadline)

        if req.navigate_to_staging:
            self.feedback(goal_handle, 'STAGING')
            frame, sx, sy, syaw = staging_pose(entry, self.get_parameter('staging_x_offset').value)
            self.navigate_to(goal_handle, deadline, frame, sx, sy, syaw)

        if 'approach_lift' in entry:
            self.raise_for_rack(goal_handle, deadline, entry['approach_lift'])
            self.latched = None               # the camera moved with the fork
        elif self.fork_at_rack_height:        # left raised by a failed rack operation
            self.set_fork(goal_handle, deadline, 0.0)
            self.fork_at_rack_height = False
            self.latched = None

        # docking_server keeps the last sighting for the blind final approach, so make sure it
        # starts from a fresh pose of THIS marker, not a leftover from an earlier dock
        t_set = time.monotonic()
        last_set = t_set
        while (self.last_pose_rx is None or self.last_pose_rx < t_set or self.latched is None):
            if goal_handle.is_cancel_requested:
                raise Failure('CANCELED', 'canceled')
            if time.monotonic() - t_set > self.perception_timeout:
                raise Failure('MARKER_NOT_FOUND', f'marker {marker_id} not seen')
            if time.monotonic() - last_set > 4.0:      # the vision node may have been respawned
                last_set = time.monotonic()
                self.set_marker_param(marker_id, goal_handle, deadline, strict=False)
            time.sleep(0.05)

        self.feedback(goal_handle, 'DOCKING')
        dock = DockRobot.Goal()
        dock.use_dock_id = True
        dock.dock_id = req.dock_id
        dock.navigate_to_staging_pose = False        # staging is done above
        res = self.run_action(self.dock_client, dock, goal_handle, deadline, 'dock_robot')
        if not res.success:
            raise Failure(dock_error_name(res.error_code),
                          f'dock_robot failed (code {res.error_code}, retries {res.num_retries})')

        self.feedback(goal_handle, 'ALIGNING')
        self.final_align(goal_handle, deadline,
                         target_from_marker(*self.latched[:2], self.latched[2], stop_distance))

        self.feedback(goal_handle, 'LIFTING')
        self.set_fork(goal_handle, deadline, lift)
        if lift > 0.02:
            self.set_carrying(True)         # the pallet is on the tines from here on
            self.carried_pallet = entry.get('pallet')
            # the pallet is below the lidar plane and only exists as painted map cells: remove
            # its slot from the costmap or the loaded footprint starts inside an "obstacle"
            self.swap_map(entry, 'map_after_pick')

        if req.undock_after and 'backoff' in entry:
            # rack: leave straight (undock_robot steers on a curve, with the load between posts)
            self.feedback(goal_handle, 'UNDOCKING')
            self.back_off(goal_handle, deadline, float(entry['backoff']))
            if 'carry_lift' in entry:         # only once the pallet is clear of the rack
                self.set_fork(goal_handle, deadline, entry['carry_lift'])
                self.fork_at_rack_height = False
        elif req.undock_after:
            self.feedback(goal_handle, 'UNDOCKING')
            und = UndockRobot.Goal()
            und.dock_type = entry.get('type', 'pallet_pick_dock')
            ures = self.run_action(self.undock_client, und, goal_handle, deadline, 'undock_robot')
            if not ures.success:
                raise Failure('UNDOCK_FAILED', f'undock failed (code {ures.error_code})')


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = PalletDockWorkflow()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
