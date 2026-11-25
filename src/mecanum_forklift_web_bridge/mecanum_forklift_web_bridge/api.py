"""
FastAPI application of the web layer: validated missions, status, e-stop, event stream.

The app only talks to a `backend` object (see ros_backend.RosBackend); tests use a fake one. Hard
rules: no endpoint accepts a velocity, a lift height or any other direct motion command; every
mutating request needs the bearer token; a mission is rejected while the e-stop is engaged, the
lift is faulted or another mission runs; the e-stop endpoint is never rate-limited and there is no
remote release.
"""
import asyncio
import json
import logging
import os
import uuid

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse

from mecanum_forklift_web_bridge.auth import token_ok
from mecanum_forklift_web_bridge.validators import ApiError, validate_mission

LOG = logging.getLogger('web_api')


def create_app(backend, token, limiter, bounds, cors_origins=(), max_body=4096,
               static_dir=None, event_period=0.5):
    """Build the FastAPI app around `backend`."""
    app = FastAPI(title='Mecanum forklift web API', version='0.1.0',
                  description='Missions, status and a software e-stop. Mutating and status '
                              'endpoints need "Authorization: Bearer <token>".')
    if cors_origins:
        app.add_middleware(CORSMiddleware, allow_origins=list(cors_origins),
                           allow_methods=['GET', 'POST'],
                           allow_headers=['Authorization', 'Content-Type'])

    def who(request):
        return request.client.host if request.client else 'unknown'

    def audit(request, what, result):
        LOG.info('command source=%s %s -> %s', who(request), what, result)

    def require_auth(request):
        if not token_ok(request.headers.get('authorization'), token):
            audit(request, f'{request.method} {request.url.path}', '401')
            raise ApiError(401, 'UNAUTHORIZED', 'missing or wrong bearer token')

    async def read_json(request):
        length = request.headers.get('content-length')
        if length is not None and length.isdigit() and int(length) > max_body:
            raise ApiError(413, 'PAYLOAD_TOO_LARGE', f'the body is limited to {max_body} bytes')
        raw = await request.body()
        if len(raw) > max_body:
            raise ApiError(413, 'PAYLOAD_TOO_LARGE', f'the body is limited to {max_body} bytes')
        try:
            return json.loads(raw or b'null')
        except ValueError as exc:
            raise ApiError(400, 'INVALID_JSON', f'the body is not valid JSON: {exc}') from exc

    @app.exception_handler(ApiError)
    async def api_error_handler(request, exc):
        headers = {'WWW-Authenticate': 'Bearer'} if exc.status == 401 else None
        return JSONResponse(exc.body(), status_code=exc.status, headers=headers)

    @app.exception_handler(Exception)
    async def unexpected_handler(request, exc):
        LOG.exception('unexpected error in %s', request.url.path)
        return JSONResponse(ApiError(500, 'INTERNAL_ERROR', 'internal error').body(),
                            status_code=500)

    @app.get('/health')
    def health():
        """Liveness (no token): the API is up; `ros` tells whether the mission server is there."""
        return {'status': 'ok', 'ros': backend.connected()}

    @app.get('/')
    def index():
        """Serve the minimal dashboard page (no token to load it; its API calls need one)."""
        path = os.path.join(static_dir or '', 'index.html')
        if not static_dir or not os.path.exists(path):
            raise ApiError(404, 'NOT_FOUND', 'no dashboard installed')
        return FileResponse(path)

    @app.get('/status')
    def status(request: Request):
        """Pose, lift, mission and e-stop state (battery and diagnostics: not available)."""
        require_auth(request)
        return backend.status()

    @app.get('/docks')
    def docks(request: Request):
        """List the dock database entries a mission may use."""
        require_auth(request)
        return backend.docks()

    @app.get('/events')
    async def events(request: Request):
        """Server-Sent Events: the /status JSON every `event_period` seconds."""
        require_auth(request)

        async def stream():
            while not await request.is_disconnected():
                yield f'data: {json.dumps(backend.status())}\n\n'
                await asyncio.sleep(event_period)

        return StreamingResponse(stream(), media_type='text/event-stream',
                                 headers={'Cache-Control': 'no-cache'})

    @app.post('/missions', status_code=202)
    async def submit_mission(request: Request):
        """Validate and submit a PALLET_TRANSPORT mission (returns its id; poll or stream)."""
        require_auth(request)
        if not limiter.allow():
            audit(request, 'POST /missions', '429')
            raise ApiError(429, 'RATE_LIMITED', 'too many requests, retry later')
        payload = await read_json(request)
        try:
            clean = validate_mission(payload, backend.docks(), bounds)
            if backend.estop_engaged():
                raise ApiError(423, 'ESTOP_ENGAGED', 'the e-stop is engaged; release it locally')
            if not backend.connected():
                raise ApiError(503, 'ROS_UNAVAILABLE', 'the mission manager is not available')
            if backend.lift_fault():
                raise ApiError(409, 'ROBOT_FAULT', 'the lift is in a fault state')
            if backend.mission_active():
                raise ApiError(409, 'BUSY', 'another mission is active')
            clean['mission_id'] = clean['mission_id'] or f'web-{uuid.uuid4().hex[:8]}'
            if backend.has_mission(clean['mission_id']):
                raise ApiError(409, 'DUPLICATE_MISSION_ID',
                               f'mission id {clean["mission_id"]!r} was already used')
            backend.submit(clean)
        except ApiError as exc:
            audit(request, 'POST /missions', f'{exc.status} {exc.code}')
            raise
        audit(request, f'POST /missions id={clean["mission_id"]}', '202')
        return {'mission_id': clean['mission_id'], 'state': 'SUBMITTED'}

    @app.get('/missions/{mission_id}')
    def get_mission(mission_id: str, request: Request):
        """State, step, result and error code of a mission submitted through this API."""
        require_auth(request)
        info = backend.mission(mission_id)
        if info is None:
            raise ApiError(404, 'NOT_FOUND', f'unknown mission {mission_id!r}')
        return info

    @app.post('/missions/{mission_id}/cancel')
    def cancel_mission(mission_id: str, request: Request):
        """Ask the mission manager to cancel an active mission (it stops the robot)."""
        require_auth(request)
        if not limiter.allow():
            raise ApiError(429, 'RATE_LIMITED', 'too many requests, retry later')
        info = backend.mission(mission_id)
        if info is None:
            raise ApiError(404, 'NOT_FOUND', f'unknown mission {mission_id!r}')
        if not info.get('active'):
            raise ApiError(409, 'NOT_ACTIVE', f'mission {mission_id!r} is not active')
        backend.cancel(mission_id)
        audit(request, f'POST /missions/{mission_id}/cancel', '200')
        return {'mission_id': mission_id, 'cancel': 'requested'}

    @app.post('/estop')
    def estop(request: Request):
        """Engage the software e-stop (zero velocity at top priority, cancel all goals)."""
        require_auth(request)                         # never rate-limited
        result = backend.engage_estop()
        audit(request, 'POST /estop', 'engaged')
        return result

    @app.post('/estop/release')
    def estop_release(request: Request):
        """Refused: the e-stop can only be released locally on the robot."""
        require_auth(request)
        audit(request, 'POST /estop/release', '403')
        raise ApiError(403, 'LOCAL_ONLY', 'release the e-stop on the robot: '
                       'ros2 service call /estop/release std_srvs/srv/Trigger')

    return app
