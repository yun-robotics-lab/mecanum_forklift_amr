"""Every dock database must be complete and point to maps that exist."""
import glob
import math
import os

import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
MAPS = os.path.join(os.path.dirname(__file__), '..', '..', 'mecanum_forklift_navigation', 'maps')
DATABASES = sorted(glob.glob(os.path.join(CONFIG, 'dock_database*.yaml')))


@pytest.mark.parametrize('path', DATABASES, ids=os.path.basename)
def test_database(path):
    with open(path, encoding='utf-8') as f:
        docks = yaml.safe_load(f)['docks']
    assert docks
    for name, entry in docks.items():
        assert len(entry['pose']) == 3, name
        for key in ('map_after_pick', 'map_after_drop'):
            if key in entry:
                assert os.path.isfile(os.path.join(MAPS, entry[key])), (name, entry[key])
        if entry.get('operation') == 'drop':
            (sx, sy, syaw), (fx, fy, fyaw) = entry['staging_pose'], entry['final_pose']
            # straight approach: staging lies behind the final pose on its heading
            distance = math.hypot(fx - sx, fy - sy)
            assert syaw == fyaw, name
            assert fx - sx == pytest.approx(distance * math.cos(fyaw), abs=0.01), name
            assert fy - sy == pytest.approx(distance * math.sin(fyaw), abs=0.01), name
        else:
            assert isinstance(entry['marker_id'], int), name
