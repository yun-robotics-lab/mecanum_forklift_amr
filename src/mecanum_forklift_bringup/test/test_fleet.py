import pytest

from mecanum_forklift_bringup.fleet import bridge_arguments, parse_fleet, port_for


def test_inline_fleet_overrides_the_file(tmp_path):
    f = tmp_path / 'r.yaml'
    f.write_text('robots:\n  - {name: a, x_pose: 1, y_pose: 2}\n')
    fleet = parse_fleet('[{"name": "b", "x_pose": 3, "y_pose": 4, "yaw": 0.5}]', str(f))
    assert [r['name'] for r in fleet] == ['b'] and fleet[0]['yaw'] == 0.5
    fleet = parse_fleet('', str(f))
    assert fleet == [{'name': 'a', 'x_pose': 1.0, 'y_pose': 2.0, 'z_pose': 0.05, 'yaw': 0.0}]


@pytest.mark.parametrize('bad', ['[]', '{}', '[{"x_pose": 1}]', '[{"name": "1a", "x_pose": 0,'
                                 ' "y_pose": 0}]', '[{"name": "a b", "x_pose": 0, "y_pose": 0}]',
                                 '[{"name": "a", "x_pose": 0, "y_pose": 0}, {"name": "a",'
                                 ' "x_pose": 1, "y_pose": 1}]', '"text"'])
def test_invalid_fleets_are_rejected(bad):
    with pytest.raises((RuntimeError, KeyError, TypeError)):
        parse_fleet(bad, '/nonexistent')


def test_default_fleet_file_has_two_distinct_robots():
    import os
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'robots.yaml')
    fleet = parse_fleet('', path)
    assert [r['name'] for r in fleet] == ['robot1', 'robot2']
    assert (fleet[0]['x_pose'], fleet[0]['y_pose']) != (fleet[1]['x_pose'], fleet[1]['y_pose'])


def test_ports_and_bridge_arguments():
    assert [port_for(8000, i) for i in range(3)] == [8000, 8001, 8002]
    args = bridge_arguments('robot2')
    assert '/robot2/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist' in args
    assert '/robot2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan' in args
    assert all(a.startswith('/robot2/') for a in args) and len(args) == 8
