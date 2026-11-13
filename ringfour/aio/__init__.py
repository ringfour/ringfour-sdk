"""ringfour.aio: the asyncio client API.

Same surface as the sync API, awaitable:

    from ringfour.aio import Robot

    async with await Robot.connect("ws://robot.local:10101") as robot:
        print(robot.name, await robot.ping())
"""

from .client import Robot
from .transport import Connection
from .ui import PowerAPI, PowerLane

__all__ = ["Robot", "Connection",
           "PowerAPI", "PowerLane"]

from .operations import Operation
__all__ += ["Operation"]
