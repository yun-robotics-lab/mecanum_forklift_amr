import os
import stat

import pytest

from mecanum_forklift_web_bridge.auth import RateLimiter, load_token, token_ok


def test_token_file_is_created_private_and_reused(tmp_path):
    path = tmp_path / 'sub' / 'token'
    first = load_token(str(path))
    assert len(first) >= 32
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600
    assert load_token(str(path)) == first


def test_short_token_rejected(tmp_path):
    path = tmp_path / 'token'
    path.write_text('short\n')
    with pytest.raises(ValueError):
        load_token(str(path))


def test_token_check():
    assert token_ok('Bearer abc123', 'abc123')
    for header in (None, '', 'abc123', 'Bearer', 'Bearer wrong', 'bearer abc123', 'Basic abc123'):
        assert not token_ok(header, 'abc123')


def test_rate_limiter_burst_then_refill():
    now = [0.0]
    limiter = RateLimiter(capacity=3, per_minute=60, clock=lambda: now[0])
    assert [limiter.allow() for _ in range(4)] == [True, True, True, False]
    now[0] = 2.0                                   # 2 s at 1 token/s
    assert [limiter.allow() for _ in range(3)] == [True, True, False]
