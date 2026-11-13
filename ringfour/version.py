"""Protocol and client version constants.

PROTOCOL_VERSION is exchanged in the hello/welcome handshake. The agent
rejects clients whose protocol version does not match (ProtocolMismatch).
Bump it only on breaking wire-format changes.
"""

PROTOCOL_VERSION = 1

CLIENT_VERSION = "0.1.0.dev0"
