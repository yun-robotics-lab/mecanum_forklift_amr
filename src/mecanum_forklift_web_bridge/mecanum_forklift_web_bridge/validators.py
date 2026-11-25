"""Request validation for the REST API (no ROS, no web framework): pure functions and ApiError."""
import math
import re

MISSION_ID_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
MISSION_TYPES = ('PALLET_TRANSPORT',)
ALLOWED_KEYS = {'mission_id', 'mission_type', 'pick_dock_id', 'drop_dock_id', 'return_pose'}


class ApiError(Exception):
    """An error with an HTTP status and a stable error code (shared JSON shape)."""

    def __init__(self, status, code, message, details=None):
        """Store the HTTP status, the error code, a message and optional details."""
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message
        self.details = details or {}

    def body(self):
        """Return the JSON body of the error response."""
        return {'error_code': self.code, 'message': self.message, 'details': self.details}


def _number(value, name, low=None, high=None):
    """Return a finite float within [low, high] or raise INVALID_PAYLOAD / POSE_OUT_OF_BOUNDS."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ApiError(400, 'INVALID_PAYLOAD', f'{name} must be a finite number')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ApiError(400, 'POSE_OUT_OF_BOUNDS', f'{name} {value} is outside [{low}, {high}]',
                       {'field': name, 'min': low, 'max': high})
    return float(value)


def validate_return_pose(pose, bounds):
    """Validate {x, y, yaw?}; bounds = (x_min, y_min, x_max, y_max) of the map. Return a dict."""
    if not isinstance(pose, dict) or not set(pose) <= {'x', 'y', 'yaw'} or 'x' not in pose \
            or 'y' not in pose:
        raise ApiError(400, 'INVALID_PAYLOAD', 'return_pose must be {x, y, yaw?}')
    x_min, y_min, x_max, y_max = bounds
    return {'x': _number(pose['x'], 'return_pose.x', x_min, x_max),
            'y': _number(pose['y'], 'return_pose.y', y_min, y_max),
            'yaw': _number(pose.get('yaw', 0.0), 'return_pose.yaw', -2 * math.pi, 2 * math.pi)}


def validate_dock(dock_id, operation, docks, field):
    """Check that dock_id names a dock database entry with the wanted operation."""
    if not isinstance(dock_id, str) or not dock_id:
        raise ApiError(400, 'INVALID_PAYLOAD', f'{field} must be a non-empty string')
    entry = docks.get(dock_id)
    if entry is None:
        raise ApiError(400, 'UNKNOWN_DOCK', f'unknown dock {dock_id!r}', {'field': field})
    if entry.get('operation', 'pick') != operation:
        raise ApiError(400, 'WRONG_DOCK_OPERATION',
                       f'dock {dock_id!r} is not a {operation} dock', {'field': field})
    if entry.get('test_only'):
        raise ApiError(400, 'UNKNOWN_DOCK', f'unknown dock {dock_id!r}', {'field': field})
    return dock_id


def validate_mission(payload, docks, bounds):
    """Validate a POST /missions body; return a clean dict (mission_id may be None)."""
    if not isinstance(payload, dict):
        raise ApiError(400, 'INVALID_PAYLOAD', 'the body must be a JSON object')
    unknown = sorted(set(payload) - ALLOWED_KEYS)
    if unknown:
        raise ApiError(400, 'INVALID_PAYLOAD', f'unknown field(s): {unknown}')
    mission_id = payload.get('mission_id')
    if mission_id is not None and (not isinstance(mission_id, str)
                                   or not MISSION_ID_RE.match(mission_id)):
        raise ApiError(400, 'INVALID_MISSION_ID', 'mission_id must match [A-Za-z0-9_-]{1,40}')
    mission_type = payload.get('mission_type', 'PALLET_TRANSPORT')
    if mission_type not in MISSION_TYPES:
        raise ApiError(400, 'INVALID_PAYLOAD', f'mission_type must be one of {MISSION_TYPES}')
    clean = {
        'mission_id': mission_id,
        'mission_type': mission_type,
        'pick_dock_id': validate_dock(payload.get('pick_dock_id'), 'pick', docks, 'pick_dock_id'),
        'drop_dock_id': validate_dock(payload.get('drop_dock_id'), 'drop', docks, 'drop_dock_id'),
        'return_pose': None,
    }
    if payload.get('return_pose') is not None:
        clean['return_pose'] = validate_return_pose(payload['return_pose'], bounds)
    return clean
