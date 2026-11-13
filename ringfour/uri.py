"""Connection string parsing.

Two transports, one code path (spec section 2):

- ``unix:///run/ringfour.sock``  - local unix domain socket
- ``ws://robot.local:7447``      - TCP WebSocket (port defaults to 7447)

``Robot()`` with no URI tries the default unix socket first, then localhost
TCP (see ``DEFAULT_CANDIDATES``).
"""

from __future__ import annotations

from dataclasses import dataclass
from urllib.parse import urlparse

from .errors import ConnectionFailed

# Default port (resolves spec Q9, Cristi 2026-06-11): 10101. The spec's
# working value 7447 collides in practice with zenoh's default TCP port
# and sits inside the DDS UDP port range (the ros2 daemon binds UDP 7447).
DEFAULT_PORT = 10101
DEFAULT_UNIX_SOCKET = "/run/ringfour.sock"

# Tried in order by Robot() when no URI is given.
DEFAULT_CANDIDATES = (
    f"unix://{DEFAULT_UNIX_SOCKET}",
    f"ws://127.0.0.1:{DEFAULT_PORT}",
)


@dataclass(frozen=True)
class ParsedUri:
    """A validated connection target.

    ``kind`` is "unix" or "tcp". For unix, ``path`` is the socket path;
    for tcp, ``host`` and ``port`` are set and ``ws_uri`` is the URI to dial.
    """

    kind: str
    path: str | None = None
    host: str | None = None
    port: int | None = None

    @property
    def ws_uri(self) -> str:
        if self.kind == "tcp":
            return f"ws://{self.host}:{self.port}"
        # websockets requires a nominal ws:// URI for unix connections too.
        return "ws://localhost/"


def parse_uri(uri: str) -> ParsedUri:
    """Parse a connection string.

    Raises ConnectionFailed on malformed input (unknown scheme, missing
    socket path or host). Does not touch the network.
    """
    parsed = urlparse(uri)
    if parsed.scheme == "unix":
        # urlparse puts the path of unix:///a/b in .path and leaves netloc
        # empty; unix://relative/path would land in netloc, which we reject.
        if parsed.netloc or not parsed.path:
            raise ConnectionFailed(
                f"malformed unix URI {uri!r}; expected unix:///absolute/path"
            )
        return ParsedUri(kind="unix", path=parsed.path)
    if parsed.scheme == "ws":
        if not parsed.hostname:
            raise ConnectionFailed(f"malformed ws URI {uri!r}; missing host")
        return ParsedUri(
            kind="tcp", host=parsed.hostname, port=parsed.port or DEFAULT_PORT
        )
    raise ConnectionFailed(
        f"unsupported URI scheme {parsed.scheme!r} in {uri!r}; "
        "use unix:///path or ws://host:port"
    )
