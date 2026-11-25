import os

from mecanum_forklift_bringup.worlds import dock_database, map_files, world_name


def test_world_name():
    assert world_name('warehouse_v2.sdf') == 'warehouse_v2'
    assert world_name('/a/b/simple_warehouse.sdf') == 'simple_warehouse'


def test_map_files_follow_world():
    amcl, nav = map_files('warehouse_v2.sdf', '/loc', '/nav')
    assert amcl == '/loc/maps/warehouse_v2.yaml'
    assert nav == '/nav/maps/warehouse_v2_nav.yaml'


def test_dock_database_is_per_world(tmp_path):
    config = tmp_path / 'config'
    config.mkdir()
    (config / 'dock_database.yaml').write_text('docks: {}')
    assert dock_database('simple_warehouse.sdf', str(tmp_path)) == os.path.join(
        str(config), 'dock_database.yaml')
    assert dock_database('warehouse_v2.sdf', str(tmp_path)) is None
    (config / 'dock_database_warehouse_v2.yaml').write_text('docks: {}')
    assert dock_database('warehouse_v2.sdf', str(tmp_path)).endswith('_warehouse_v2.yaml')
