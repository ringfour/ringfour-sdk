"""Local discovered references; accessing a signal never makes an RPC."""
from .models.measurements import SignalCondition
from .errors import CapabilityNotSupported

class Signal:
    def __init__(self, robot_id, path, metadata):
        self.robot_id,self.path,self.metadata=robot_id,path,metadata
    @property
    def unit(self):return self.metadata.get('unit')
    def below(self, threshold, *, max_age=.5):
        return SignalCondition(robot_id=self.robot_id,path=self.path,comparison='below',threshold=threshold,max_age=max_age,frame=self.metadata.get("frame"))
    def above(self, threshold, *, max_age=.5):
        return SignalCondition(robot_id=self.robot_id,path=self.path,comparison='above',threshold=threshold,max_age=max_age,frame=self.metadata.get("frame"))

class Signals:
    def __init__(self, robot_id, inventory, prefix=''):
        self._robot_id,self._inventory,self._prefix=robot_id,inventory,prefix
    def __getattr__(self, name):return self[name]
    def __getitem__(self, name):
        path=self._prefix+name
        if path in self._inventory:return Signal(self._robot_id,path,self._inventory[path])
        if any(key.startswith(path+'.') for key in self._inventory):
            return Signals(self._robot_id,self._inventory,path+'.')
        raise CapabilityNotSupported(f'signal {path!r} is not declared on this robot')
