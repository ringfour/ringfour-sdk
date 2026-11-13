# Ringfour SDK

Pure Python client for the Ringfour agent. Installable distribution: `ringfour-sdk`; Python import: `ringfour`. The core needs websockets, NumPy and Pydantic, with no ROS installation.

From this repository:

```sh
uv pip install -e '.[dev]'
uv build
```

```python
from ringfour import Robot

with Robot("ws://127.0.0.1:10101") as robot:
    print(robot.name, robot.capabilities)
```

Use a running agent configured by the robot integrator. Network operation permission is a deployment policy; a connection alone does not grant motion authority. TCP arrays use NET, allowing the SDK to run natively on macOS.

- [API reference](docs/api-reference.md)
- [Generated types](docs/types-reference.md) and [wire schema](protocol.json)
- [Examples, recipes and course](../missions/README.md)
- [Platform tests and dashboard](../ringfour-testing/README.md)
- [Current hardening contracts](../ringfour-testing/docs/milestone-c.md) and [migration notes](../ringfour-testing/MIGRATION.md)

Teaching and mission code live in `../missions`, hardening acceptance records in `../ringfour-testing`. This repository owns the client, shared models, protocol/schema generators and current API reference. Optional client-side solvers are available through `ringfour-sdk[kinematics]`.
