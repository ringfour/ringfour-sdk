"""Explicit integration diagnostics, outside ordinary mission metadata."""
from types import MappingProxyType

class RobotIntegration:
    def __init__(self, robot):self._robot=robot
    @property
    def controllers(self):
        robot=getattr(self._robot,'_robot',self._robot)
        values=robot._welcome.features.get('integration',{}).get('controllers',{})
        return MappingProxyType({name:MappingProxyType(dict(roles)) for name,roles in values.items()})
    @property
    def slam_backend(self):
        robot=getattr(self._robot,'_robot',self._robot)
        return robot._welcome.features.get('integration',{}).get('slam_backend')

    def group_controllers(self,name):return self.controllers.get(name,MappingProxyType({}))

class GroupIntegration:
    def __init__(self,group):self._group=group
    def control(self,*,raw):return self._group._control(raw=raw)
