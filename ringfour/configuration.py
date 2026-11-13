"""Build named values from the discovered model, without ROS dependencies."""
from .models.execution import JointConfiguration,JointValue
from .errors import CapabilityNotSupported,ModelChanged


def configuration(group, positions):
    from .kinematics import _resolve
    info,description,robot=_resolve(group)
    if set(positions)!=set(info.joints):raise ValueError('configuration requires every group joint exactly once')
    values=[]
    for name in info.joints:
        joint=description.joints[name]
        unit={'revolute':'rad','continuous':'rad','prismatic':'m'}.get(joint.type)
        if unit is None:raise CapabilityNotSupported(f'joint {name} has no supported native units')
        lo,hi=joint.limits or (None,None)
        values.append(JointValue(name=name,position=positions[name],unit=unit,lower=lo,upper=hi))
    return JointConfiguration(robot=robot.name,group=group.name,model_revision=description.sha,joints=tuple(values))


def validate_configuration(group,value):
    from .kinematics import _resolve
    info,description,robot=_resolve(group)
    if not isinstance(value,JointConfiguration):raise TypeError('expected a JointConfiguration from group.configuration() or kin.ik()')
    if value.robot!=robot.name or value.group!=group.name:raise ValueError('configuration belongs to another robot or group')
    if value.model_revision!=description.sha:raise ModelChanged('configuration uses an old robot model')
    rebuilt=configuration(group,value.positions)
    if value!=rebuilt:raise ValueError('configuration joint units/bounds differ from the discovered model')
    return value.vector(info.joints)
