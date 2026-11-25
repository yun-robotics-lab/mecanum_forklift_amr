"""Bearer-token authentication and a token-bucket rate limiter (no ROS, no web framework)."""
import hmac
import os
import secrets
import time


def load_token(path):
    """Return the token stored in `path`; create the file (mode 0600) with a new one if missing."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as f:
            f.write(secrets.token_urlsafe(32) + '\n')
    with open(path, encoding='utf-8') as f:
        token = f.read().strip()
    if len(token) < 16:
        raise ValueError(f'token in {path} is too short (need at least 16 characters)')
    return token


def token_ok(header, token):
    """Return True if `header` is 'Bearer <token>' with the right token (constant time)."""
    if not header or not header.startswith('Bearer '):
        return False
    return hmac.compare_digest(header[len('Bearer '):].encode(), token.encode())


class RateLimiter:
    """Token bucket: `capacity` burst, refilled at `per_minute` tokens per minute."""

    def __init__(self, capacity=5, per_minute=10, clock=time.monotonic):
        """Create a full bucket."""
        self.capacity = float(capacity)
        self.rate = per_minute / 60.0
        self.clock = clock
        self.tokens = self.capacity
        self.last = clock()

    def allow(self):
        """Take one token; return False if the bucket is empty."""
        now = self.clock()
        self.tokens = min(self.capacity, self.tokens + (now - self.last) * self.rate)
        self.last = now
        if self.tokens >= 1.0:
            self.tokens -= 1.0
            return True
        return False
