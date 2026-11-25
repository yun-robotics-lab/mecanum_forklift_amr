import math

import pytest

from mecanum_forklift_web_bridge.validators import ApiError, validate_mission

DOCKS = {'pick1': {'operation': 'pick'}, 'drop1': {'operation': 'drop'}, 'pick2': {},
         'hidden': {'test_only': True}}
BOUNDS = (-3.04, -2.54, 3.01, 2.56)


def ok(**kw):
    payload = {'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1'}
    payload.update(kw)
    return validate_mission(payload, DOCKS, BOUNDS)


def code(payload):
    with pytest.raises(ApiError) as exc:
        validate_mission(payload, DOCKS, BOUNDS)
    return exc.value.status, exc.value.code


def test_minimal_valid_mission():
    clean = ok()
    assert clean['mission_type'] == 'PALLET_TRANSPORT' and clean['return_pose'] is None
    assert clean['mission_id'] is None


def test_dock_without_operation_counts_as_pick():
    assert ok(pick_dock_id='pick2')['pick_dock_id'] == 'pick2'


def test_valid_return_pose_defaults_yaw():
    assert ok(return_pose={'x': 0, 'y': 0})['return_pose'] == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


@pytest.mark.parametrize('payload', [[], 'x', None, 3])
def test_body_must_be_an_object(payload):
    assert code(payload) == (400, 'INVALID_PAYLOAD')


def test_unknown_fields_rejected():
    assert code({'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1', 'speed': 5}) == \
        (400, 'INVALID_PAYLOAD')


@pytest.mark.parametrize('mission_id', ['', 'a b', 'x' * 41, 'a/b', 7, '../etc'])
def test_bad_mission_ids(mission_id):
    assert code({'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1', 'mission_id': mission_id}) \
        == (400, 'INVALID_MISSION_ID')


def test_mission_id_boundaries():
    assert ok(mission_id='a')['mission_id'] == 'a'
    assert ok(mission_id='x' * 40)['mission_id'] == 'x' * 40


def test_bad_mission_type():
    assert code({'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1', 'mission_type': 'PATROL'}) \
        == (400, 'INVALID_PAYLOAD')


@pytest.mark.parametrize('payload,expected', [
    ({'drop_dock_id': 'drop1'}, (400, 'INVALID_PAYLOAD')),
    ({'pick_dock_id': 'nope', 'drop_dock_id': 'drop1'}, (400, 'UNKNOWN_DOCK')),
    ({'pick_dock_id': 'drop1', 'drop_dock_id': 'drop1'}, (400, 'WRONG_DOCK_OPERATION')),
    ({'pick_dock_id': 'pick1', 'drop_dock_id': 'pick1'}, (400, 'WRONG_DOCK_OPERATION')),
    ({'pick_dock_id': 'hidden', 'drop_dock_id': 'drop1'}, (400, 'UNKNOWN_DOCK')),
    ({'pick_dock_id': 5, 'drop_dock_id': 'drop1'}, (400, 'INVALID_PAYLOAD')),
])
def test_dock_checks(payload, expected):
    assert code(payload) == expected


@pytest.mark.parametrize('pose', [{'x': 3.01, 'y': 2.56}, {'x': -3.04, 'y': -2.54},
                                  {'x': 0, 'y': 0, 'yaw': 2 * math.pi}])
def test_pose_boundaries_are_inside(pose):
    assert ok(return_pose=pose)['return_pose'] is not None


@pytest.mark.parametrize('pose', [{'x': 3.02, 'y': 0}, {'x': 0, 'y': -2.55}, {'x': 0, 'y': 0,
                                                                              'yaw': 7.0}])
def test_pose_out_of_bounds(pose):
    assert code({'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1', 'return_pose': pose}) == \
        (400, 'POSE_OUT_OF_BOUNDS')


@pytest.mark.parametrize('pose', [{'x': 'a', 'y': 0}, {'x': math.nan, 'y': 0},
                                  {'x': math.inf, 'y': 0}, {'x': True, 'y': 0}, {'x': 1},
                                  {'x': 0, 'y': 0, 'z': 1}, 'home', []])
def test_pose_malformed(pose):
    assert code({'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1', 'return_pose': pose}) == \
        (400, 'INVALID_PAYLOAD')
