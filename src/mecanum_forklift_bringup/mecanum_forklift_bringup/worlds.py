"""Which map and dock database files belong to a Gazebo world (pure functions, no ROS)."""
import os


def world_name(world):
    """'warehouse_v2.sdf' or a full path -> 'warehouse_v2'."""
    return os.path.splitext(os.path.basename(world))[0]


def map_files(world, localization_share, navigation_share):
    """
    Return (AMCL map, Nav2 costmap map) yaml paths for a world.

    Naming: localization maps/<world>.yaml and navigation maps/<world>_nav.yaml.
    """
    name = world_name(world)
    return (os.path.join(localization_share, 'maps', name + '.yaml'),
            os.path.join(navigation_share, 'maps', name + '_nav.yaml'))


def dock_database(world, docking_share):
    """
    Return the dock database path of a world, or None if the world has none.

    simple_warehouse uses config/dock_database.yaml, other worlds
    config/dock_database_<world>.yaml. A database is never shared between worlds: its poses and
    maps are only valid in its own world.
    """
    name = world_name(world)
    file = 'dock_database.yaml' if name == 'simple_warehouse' else f'dock_database_{name}.yaml'
    path = os.path.join(docking_share, 'config', file)
    return path if os.path.isfile(path) else None
