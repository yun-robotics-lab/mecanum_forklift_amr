"""Pure helpers for the DockPallet workflow (no ROS): database lookup and error mapping."""
import math

import yaml

# opennav_docking_msgs/DockRobot error codes
DOCK_NOT_IN_DB = 901
DOCK_NOT_VALID = 902
FAILED_TO_STAGE = 903
FAILED_TO_DETECT_DOCK = 904
FAILED_TO_CONTROL = 905

DOCK_ERRORS = {
    DOCK_NOT_IN_DB: 'DOCK_NOT_IN_DB',
    DOCK_NOT_VALID: 'DOCK_FAILED',
    FAILED_TO_STAGE: 'STAGING_FAILED',
    FAILED_TO_DETECT_DOCK: 'MARKER_NOT_FOUND',
    FAILED_TO_CONTROL: 'CONTROL_FAILED',
}


def dock_error_name(code):
    """Map a DockRobot error code to a DockPallet error_code string."""
    return DOCK_ERRORS.get(int(code), 'DOCK_FAILED')


def load_docks(path):
    """Return the `docks` mapping of a dock database file ({} if missing)."""
    with open(path, encoding='utf-8') as f:
        return (yaml.safe_load(f) or {}).get('docks', {}) or {}


def lookup_dock(docks, dock_id):
    """Return (marker_id, lift_height) for dock_id, or None if unknown or incomplete."""
    entry = docks.get(dock_id)
    if not entry or 'marker_id' not in entry:
        return None
    return int(entry['marker_id']), float(entry.get('lift', 0.0))


def progress_of(state):
    """Coarse progress (percent) for a workflow state."""
    return {'SET_MARKER': 5.0, 'STAGING': 15.0, 'DOCKING': 30.0, 'ALIGNING': 60.0,
            'LIFTING': 80.0, 'LOWERING': 80.0, 'UNDOCKING': 92.0, 'DONE': 100.0}.get(state, 0.0)


def polygon_from_flat(values):
    """Convert [x1, y1, x2, y2, ...] to [(x1, y1), ...]; raise ValueError if malformed."""
    if len(values) < 6 or len(values) % 2:
        raise ValueError('a footprint needs at least 3 points (x, y pairs)')
    return [(float(values[i]), float(values[i + 1])) for i in range(0, len(values), 2)]


def to_frame(target, robot_src, robot_dst):
    """
    Express `target` (x, y, yaw) of a source frame in a destination frame.

    robot_src and robot_dst are the robot pose in the two frames at the same instant. The result
    stays fixed in the destination frame even if the source frame (map, AMCL) later moves.
    """
    turn = robot_dst[2] - robot_src[2]
    dx, dy = target[0] - robot_src[0], target[1] - robot_src[1]
    yaw = target[2] + turn
    return (robot_dst[0] + dx * math.cos(turn) - dy * math.sin(turn),
            robot_dst[1] + dx * math.sin(turn) + dy * math.cos(turn),
            math.atan2(math.sin(yaw), math.cos(yaw)))


def wrong_pallet(entry, carried):
    """Return True if a drop entry is only valid for another pallet than the carried one."""
    wanted = entry.get('pallet')
    return bool(wanted and carried and wanted != carried)


def staging_pose(entry, x_offset):
    """Return (frame, x, y, yaw): the dock pose shifted by x_offset along its yaw."""
    x, y, yaw = (float(v) for v in entry['pose'])
    return (entry.get('frame', 'map'), x + x_offset * math.cos(yaw), y + x_offset * math.sin(yaw),
            yaw)
