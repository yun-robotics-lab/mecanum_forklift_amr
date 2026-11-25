"""Rack docks: step order of the workflow, and fork / chassis clearances (no ROS, no Gazebo)."""
import math
import os
import re
from types import SimpleNamespace

from mecanum_forklift_docking.dock_core import load_docks, to_frame, wrong_pallet
from mecanum_forklift_docking.workflow_node import Failure, PalletDockWorkflow

import pytest

SRC = os.path.join(os.path.dirname(__file__), '..', '..')
DOCKS = load_docks(os.path.join(
    SRC, 'mecanum_forklift_docking', 'config', 'dock_database_warehouse_v2.yaml'))

# robot geometry (URDF + fork.stl), m
CHASSIS_FRONT = 0.2070      # base_link -> front face of the chassis collision box
CARRIAGE_FRONT = 0.228      # base_link -> front face of the fork carriage
TINE_UNDERSIDE, TINE_TOP = 0.010, 0.016     # above ground at lift 0
CARRY_CENTRE = 0.335        # base_link -> centre of a carried pallet
# mini_pallet
POCKET_HEIGHT, PALLET_DEPTH = 0.040, 0.20   # deck underside above the pallet base; depth


def test_to_frame():
    assert to_frame((2.0, 3.0, 0.5), (1.0, 1.0, 0.0), (1.0, 1.0, 0.0)) == pytest.approx(
        (2.0, 3.0, 0.5))
    # same robot, seen in a frame rotated by +90 deg: 1 m ahead of the robot stays 1 m ahead
    x, y, yaw = to_frame((2.0, 0.0, 0.0), (1.0, 0.0, 0.0), (5.0, 5.0, math.pi / 2))
    assert (x, y, yaw) == pytest.approx((5.0, 6.0, math.pi / 2))


def test_wrong_pallet():
    assert wrong_pallet({'pallet': 'pallet_1'}, 'pallet_rack_1')
    assert not wrong_pallet({'pallet': 'pallet_1'}, 'pallet_1')
    assert not wrong_pallet({}, 'pallet_rack_1')            # rack drops take any pallet
    assert not wrong_pallet({'pallet': 'pallet_1'}, None)   # databases without `pallet`


class Recorder(SimpleNamespace):
    """Stand-in for the node: records what a sequence asks the robot to do."""

    def __init__(self, **state):
        super().__init__(log=[], carrying=False, carried_pallet=None, map_swapped=False,
                         fork_at_rack_height=False, latched=None, last_pose_rx=None,
                         stop_distance=0.245, perception_timeout=1.0, docks=DOCKS)
        self.__dict__.update(state)

    def add(self, *item):
        self.log.append(item)

    def feedback(self, goal_handle, state):
        pass

    def see_marker(self):
        self.last_pose_rx, self.latched = float('inf'), (-0.8, 2.2, math.pi / 2)

    def set_fork(self, goal_handle, deadline, height):
        self.add('fork', height)

    raise_for_rack = PalletDockWorkflow.raise_for_rack

    def freeze_in_odom(self, target, frame):
        self.add('freeze', frame)
        return target

    def navigate_to(self, goal_handle, deadline, *pose):
        self.add('navigate')

    def final_align(self, goal_handle, deadline, target, frame='odom'):
        self.add('align', frame)

    def back_off(self, goal_handle, deadline, distance):
        self.add('back_off', distance)

    def set_carrying(self, carrying):
        self.carrying = carrying
        self.add('carrying', carrying)

    def swap_map(self, entry, key):
        if entry.get(key):
            self.add('map', entry[key])

    def load_nav_map(self, path):
        self.add('restore_map')
        return True

    def set_marker_param(self, *args, **kwargs):
        pass

    def run_action(self, client, goal, goal_handle, deadline, name, full=False):
        self.add(name)
        return SimpleNamespace(success=True)

    def get_parameter(self, name):
        return SimpleNamespace(value=-0.25)

    dock_client = undock_client = None


def request(dock_id):
    return SimpleNamespace(dock_id=dock_id, navigate_to_staging=True, undock_after=True)


class Handle:
    """Goal handle: while the sequence waits (and polls cancel), the camera sees the marker."""

    def __init__(self, robot):
        self.robot = robot

    @property
    def is_cancel_requested(self):
        self.robot.see_marker()
        return False


def pick(robot, dock_id):
    PalletDockWorkflow.pick_sequence(robot, Handle(robot), request(dock_id), 1e12, DOCKS[dock_id])


def drop(robot, dock_id):
    PalletDockWorkflow.drop_sequence(robot, Handle(robot), request(dock_id), 1e12, DOCKS[dock_id])


def steps(robot):
    return [s for s in robot.log if s[0] != 'carrying']


def test_rack_pick_order():
    robot = Recorder(map_swapped=True)
    pick(robot, 'rack_pick_1')
    # raised before the approach, lowered to carry height only after backing out;
    # the current map is kept
    assert steps(robot) == [('navigate',), ('fork', 0.067), ('dock_robot',), ('align', 'odom'),
                            ('fork', 0.109), ('back_off', 0.40), ('fork', 0.06)]
    assert robot.carrying and robot.carried_pallet == 'pallet_rack_1' and robot.map_swapped
    assert not robot.fork_at_rack_height


def test_rack_drop_order():
    robot = Recorder(carrying=True, carried_pallet='pallet_1')
    drop(robot, 'rack_drop_2')
    assert steps(robot) == [('navigate',), ('freeze', 'map'), ('fork', 0.109), ('align', 'odom'),
                            ('fork', 0.067), ('back_off', 0.40), ('fork', 0.0)]
    assert not robot.carrying and robot.carried_pallet is None


def test_floor_drop_refuses_rack_pallet_before_moving():
    robot = Recorder(carrying=True, carried_pallet='pallet_rack_1')
    with pytest.raises(Failure) as error:
        drop(robot, 'pallet_drop_1')
    assert error.value.code == 'WRONG_PALLET' and not robot.log


def test_floor_pick_does_not_move_the_fork_first():
    robot = Recorder()
    pick(robot, 'pallet_pick_1')
    assert steps(robot)[:3] == [('navigate',), ('dock_robot',), ('align', 'odom')]
    assert steps(robot)[-1] == ('undock_robot',)


def test_floor_pick_after_failed_rack_operation_lowers_fork():
    robot = Recorder(fork_at_rack_height=True)
    pick(robot, 'pallet_pick_1')
    assert steps(robot)[1] == ('fork', 0.0)


def world_pose(name):
    with open(os.path.join(SRC, 'mecanum_forklift_gazebo', 'worlds', 'warehouse_v2.sdf'),
              encoding='utf-8') as f:
        found = re.search(rf'<name>{name}</name>\s*<pose>([^<]*)</pose>', f.read())
    return [float(v) for v in found[1].split()]


def test_rack_geometry():
    rack_y = world_pose('rack_a1')[1]
    deck_front, deck_top = rack_y - 0.18, 0.060          # models/pallet_rack
    pallet_y = world_pose('pallet_rack_1')[1]
    marker_y = pallet_y - 0.09                           # cargo_box_marked_1 at yaw 90 deg
    pick = DOCKS['rack_pick_1']
    base_at_pick = marker_y - 0.245
    assert pick['pose'][1] == pytest.approx(base_at_pick - 0.15)
    for entry, base_y in [(pick, base_at_pick)] + [
            (DOCKS[n], DOCKS[n]['final_pose'][1]) for n in ('rack_drop_1', 'rack_drop_2')]:
        assert deck_front - (base_y + CHASSIS_FRONT) >= 0.05           # chassis never hits
        assert base_y + CARRIAGE_FRONT < pallet_y - PALLET_DEPTH / 2   # carriage stops short
        # backed out: the whole carried pallet is clear of the rack
        assert base_y - entry['backoff'] + CARRY_CENTRE + PALLET_DEPTH / 2 < deck_front - 0.15
    # a pallet placed by a drop stands where the pick expects it, centre of mass on the deck
    assert DOCKS['rack_drop_1']['final_pose'][1] + CARRY_CENTRE == pytest.approx(pallet_y)
    assert pallet_y - deck_front >= 0.05
    # fork windows: the tines fit between the deck and the underside of the pallet deck
    for lift in (pick['approach_lift'], DOCKS['rack_drop_1']['lift']):
        assert TINE_UNDERSIDE + lift >= deck_top + 0.01
        assert TINE_TOP + lift <= deck_top + POCKET_HEIGHT - 0.01
    # lifted: the pallet base (tine top - pocket height) is clear above the deck
    for lift in (pick['lift'], DOCKS['rack_drop_1']['approach_lift']):
        assert TINE_TOP + lift - POCKET_HEIGHT >= deck_top + 0.02
        assert lift <= 0.12
