import math

import pytest

from mecanum_forklift_docking.dock_core import (
    dock_error_name, load_docks, lookup_dock, polygon_from_flat, progress_of, staging_pose)


def test_error_mapping():
    assert dock_error_name(904) == 'MARKER_NOT_FOUND'
    assert dock_error_name(905) == 'CONTROL_FAILED'
    assert dock_error_name(903) == 'STAGING_FAILED'
    assert dock_error_name(12345) == 'DOCK_FAILED'


def test_lookup(tmp_path):
    f = tmp_path / 'db.yaml'
    f.write_text('docks:\n  a: {marker_id: 3, lift: 0.05}\n  b: {type: x}\n')
    docks = load_docks(str(f))
    assert lookup_dock(docks, 'a') == (3, 0.05)
    assert lookup_dock(docks, 'b') is None        # no marker id
    assert lookup_dock(docks, 'zzz') is None


def test_progress_monotonic():
    order = ['SET_MARKER', 'STAGING', 'DOCKING', 'ALIGNING', 'LIFTING', 'UNDOCKING', 'DONE']
    values = [progress_of(s) for s in order]
    assert values == sorted(values) and values[-1] == 100.0


def test_polygon_from_flat():
    pts = polygon_from_flat([0.4, 0.15, 0.4, -0.15, -0.2, -0.15, -0.2, 0.15])
    assert pts[0] == (0.4, 0.15) and len(pts) == 4
    with pytest.raises(ValueError):
        polygon_from_flat([0.0, 0.0, 1.0, 1.0])            # only 2 points
    with pytest.raises(ValueError):
        polygon_from_flat([0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0])   # odd length


def test_staging_pose():
    frame, x, y, yaw = staging_pose({'frame': 'odom', 'pose': [0.415, 0.0, 0.0]}, -0.25)
    assert frame == 'odom' and abs(x - 0.165) < 1e-9 and y == 0.0 and yaw == 0.0
    _, x, y, yaw = staging_pose({'pose': [1.0, 2.0, math.pi / 2]}, -0.5)
    assert abs(x - 1.0) < 1e-9 and abs(y - 1.5) < 1e-9
