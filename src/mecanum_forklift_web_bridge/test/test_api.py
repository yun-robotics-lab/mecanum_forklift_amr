import json

import pytest
from fastapi.testclient import TestClient

from mecanum_forklift_web_bridge.api import create_app
from mecanum_forklift_web_bridge.auth import RateLimiter
from mecanum_forklift_web_bridge.validators import ApiError

TOKEN = 'secret-token-0123456789'
AUTH = {'Authorization': f'Bearer {TOKEN}'}
DOCKS = {'pick1': {'operation': 'pick'}, 'drop1': {'operation': 'drop'}}
BOUNDS = (-3.0, -2.5, 3.0, 2.5)
GOOD = {'pick_dock_id': 'pick1', 'drop_dock_id': 'drop1'}


class Fake:
    def __init__(self):
        self.up = True
        self.estop = False
        self.fault = False
        self.active = False
        self.missions = {}
        self.cancelled = []
        self.estops = 0

    def connected(self):
        return self.up

    def status(self):
        return {'estop': self.estop, 'mission': {'active': self.active}}

    def docks(self):
        return DOCKS

    def estop_engaged(self):
        return self.estop

    def lift_fault(self):
        return self.fault

    def mission_active(self):
        return self.active

    def has_mission(self, mission_id):
        return mission_id in self.missions

    def submit(self, clean):
        self.missions[clean['mission_id']] = {'mission_id': clean['mission_id'], 'active': True}

    def mission(self, mission_id):
        return self.missions.get(mission_id)

    def cancel(self, mission_id):
        self.cancelled.append(mission_id)

    def engage_estop(self):
        self.estops += 1
        self.estop = True
        return {'estop': 'engaged'}


@pytest.fixture
def rig():
    backend = Fake()
    app = create_app(backend, TOKEN, RateLimiter(capacity=50, per_minute=600), BOUNDS,
                     max_body=300)
    return backend, TestClient(app, raise_server_exceptions=False)


def post(client, body, headers=AUTH, path='/missions'):
    raw = body if isinstance(body, (str, bytes)) else json.dumps(body)
    return client.post(path, content=raw, headers={**(headers or {}),
                                                   'Content-Type': 'application/json'})


def test_health_needs_no_token_and_reports_ros(rig):
    backend, client = rig
    assert client.get('/health').json() == {'status': 'ok', 'ros': True}
    backend.up = False
    assert client.get('/health').json()['ros'] is False


@pytest.mark.parametrize('path', ['/status', '/docks', '/missions/x', '/events'])
def test_reads_need_the_token(rig, path):
    _, client = rig
    for headers in (None, {'Authorization': 'Bearer wrong'}):
        resp = client.get(path, headers=headers)
        assert resp.status_code == 401 and resp.json()['error_code'] == 'UNAUTHORIZED'


def test_valid_mission_is_accepted_with_a_generated_id(rig):
    backend, client = rig
    resp = post(client, GOOD)
    assert resp.status_code == 202 and resp.json()['state'] == 'SUBMITTED'
    assert resp.json()['mission_id'].startswith('web-') and len(backend.missions) == 1


def test_no_token_means_no_side_effect(rig):
    backend, client = rig
    assert post(client, GOOD, headers=None).status_code == 401
    assert backend.missions == {}


@pytest.mark.parametrize('body,status,code', [
    ('not json', 400, 'INVALID_JSON'),
    ({'pick_dock_id': 'pick1'}, 400, 'INVALID_PAYLOAD'),
    ({**GOOD, 'drop_dock_id': 'pick1'}, 400, 'WRONG_DOCK_OPERATION'),
    ({**GOOD, 'pick_dock_id': 'x'}, 400, 'UNKNOWN_DOCK'),
    ({**GOOD, 'mission_id': 'bad id'}, 400, 'INVALID_MISSION_ID'),
    ({**GOOD, 'return_pose': {'x': 9, 'y': 0}}, 400, 'POSE_OUT_OF_BOUNDS'),
    ({**GOOD, 'cmd_vel': {'x': 1}}, 400, 'INVALID_PAYLOAD'),
    ({**GOOD, 'pad': 'x' * 400}, 413, 'PAYLOAD_TOO_LARGE'),
])
def test_invalid_requests_have_no_side_effect(rig, body, status, code):
    backend, client = rig
    resp = post(client, body)
    assert resp.status_code == status and resp.json()['error_code'] == code
    assert backend.missions == {}


def test_state_rejections(rig):
    backend, client = rig
    backend.estop = True
    assert (post(client, GOOD).status_code, post(client, GOOD).json()['error_code']) == \
        (423, 'ESTOP_ENGAGED')
    backend.estop, backend.up = False, False
    assert post(client, GOOD).status_code == 503
    backend.up, backend.fault = True, True
    assert post(client, GOOD).json()['error_code'] == 'ROBOT_FAULT'
    backend.fault, backend.active = False, True
    assert post(client, GOOD).json()['error_code'] == 'BUSY'
    backend.active = False
    assert post(client, {**GOOD, 'mission_id': 'a1'}).status_code == 202
    resp = post(client, {**GOOD, 'mission_id': 'a1'})
    assert (resp.status_code, resp.json()['error_code']) == (409, 'DUPLICATE_MISSION_ID')


def test_get_and_cancel_mission(rig):
    backend, client = rig
    post(client, {**GOOD, 'mission_id': 'm1'})
    assert client.get('/missions/m1', headers=AUTH).json()['active'] is True
    assert client.get('/missions/nope', headers=AUTH).status_code == 404
    assert client.post('/missions/nope/cancel', headers=AUTH).status_code == 404
    assert client.post('/missions/m1/cancel', headers=AUTH).json()['cancel'] == 'requested'
    assert backend.cancelled == ['m1']
    backend.missions['m1']['active'] = False
    assert client.post('/missions/m1/cancel', headers=AUTH).json()['error_code'] == 'NOT_ACTIVE'


def test_rate_limit_applies_to_missions_but_never_to_estop():
    backend = Fake()
    app = create_app(backend, TOKEN, RateLimiter(capacity=2, per_minute=1), BOUNDS)
    client = TestClient(app, raise_server_exceptions=False)
    codes = [post(client, {'bad': 1}).status_code for _ in range(3)]
    assert codes == [400, 400, 429]
    for _ in range(10):
        assert client.post('/estop', headers=AUTH).status_code == 200
    assert backend.estops == 10


def test_estop_needs_token_and_release_is_refused(rig):
    backend, client = rig
    assert client.post('/estop').status_code == 401 and backend.estops == 0
    assert client.post('/estop', headers=AUTH).json() == {'estop': 'engaged'}
    resp = client.post('/estop/release', headers=AUTH)
    assert resp.status_code == 403 and resp.json()['error_code'] == 'LOCAL_ONLY'
    assert backend.estop is True                      # still engaged


def test_no_endpoint_accepts_motion_commands(rig):
    _, client = rig
    paths = set(client.get('/openapi.json').json()['paths'])
    assert not any(w in p for p in paths for w in ('cmd_vel', 'velocity', 'lift', 'teleop'))


def test_internal_errors_are_hidden(rig):
    backend, client = rig
    backend.status = lambda: 1 / 0
    resp = client.get('/status', headers=AUTH)
    assert resp.status_code == 500 and resp.json()['error_code'] == 'INTERNAL_ERROR'
    assert 'ZeroDivision' not in resp.text


def test_backend_api_error_is_passed_through(rig):
    backend, client = rig

    def boom(clean):
        raise ApiError(503, 'ROS_UNAVAILABLE', 'gone')
    backend.submit = boom
    assert post(client, GOOD).status_code == 503
