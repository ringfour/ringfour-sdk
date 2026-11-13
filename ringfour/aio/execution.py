"""Typed operation producers shared by the normal SDK motion methods."""
from ..models.execution import (NavigationResult,NavigationProgress,TrajectoryResult,
    TrajectoryProgress,JointConfiguration,JointValue,JointResidual,StopReport)
from ..errors import CompletionUnverified,ModelChanged
from .operations import Operation


def stop_report(resource,evidence):
    if not evidence or not evidence.get('verified'):
        raise CompletionUnverified('operation lacks verified stopping evidence')
    return StopReport(resource=resource,verified=True,criterion=evidence['criterion'],
        physical_stop=evidence['physical_stop'],elapsed=evidence.get('elapsed'),dwell=evidence.get('dwell'))


def navigation_operation(robot,task):
    def result(identifier,value):
        try:
            evidence=value['arrival_verification']
            if not evidence['verified']:raise ValueError('unverified arrival')
            return NavigationResult(operation_id=identifier,final_pose=evidence['final_pose'],
                position_error=evidence['position_error'],orientation_error=evidence['orientation_error'],
                position_tolerance=evidence['position_tolerance'],orientation_tolerance=evidence['orientation_tolerance'],
                stop=stop_report('mobility',value['stop_verification']))
        except (KeyError,ValueError,TypeError) as exc:raise CompletionUnverified('arrival result is incomplete',details={'operation_id':identifier}) from exc
    def progress(task):
        data=task.data
        if not data:return None
        return NavigationProgress(pose=data.get('pose'),remaining_distance=data.get('distance_remaining'))
    return Operation(task,result,progress,resource='mobility',cleanup_timeout=robot._welcome.features['operation_cleanup_timeout'])


def trajectory_operation(robot,task,trajectory):
    template=trajectory.configurations[-1]
    def result(identifier,value):
        try:
            if value['model_revision']!=template.model_revision:raise ModelChanged('trajectory result belongs to a different model')
            if set(value['positions'])!=set(template.positions) or set(value['residual'])!=set(template.positions):raise ValueError('missing joint evidence')
            joints=tuple(JointValue(name=j.name,position=value['positions'][j.name],unit=j.unit,lower=j.lower,upper=j.upper) for j in template.joints)
            configuration=JointConfiguration(robot=template.robot,group=template.group,model_revision=template.model_revision,joints=joints)
            residuals=tuple(JointResidual(name=j.name,error=value['residual'][j.name],unit=j.unit,tolerance=value['goal_tolerances'][j.name]) for j in template.joints)
            if any(abs(r.error)>r.tolerance for r in residuals):raise ValueError('out-of-tolerance endpoint')
            return TrajectoryResult(operation_id=identifier,configuration=configuration,residuals=residuals,
                stop=stop_report(template.group,value['stop_verification']))
        except (KeyError,ValueError,TypeError) as exc:raise CompletionUnverified('trajectory result is incomplete',details={'operation_id':identifier}) from exc
    def progress(task):
        data=task.data or {};positions=None
        names=data.get('joints');actual=data.get('actual')
        if names and actual and len(names)==len(actual) and set(names)==set(template.positions):
            units={j.name:j.unit for j in template.joints}
            positions=tuple(JointValue(name=n,position=p,unit=units[n]) for n,p in zip(names,actual))
        if positions is None and task.progress is None:return None
        return TrajectoryProgress(positions=positions,elapsed_fraction=task.progress,
            fraction_basis='elapsed_time' if task.progress is not None else None)
    return Operation(task,result,progress,resource=template.group,cleanup_timeout=robot._welcome.features['operation_cleanup_timeout'])


def route_operation(robot,task,expected_count):
    from ..models.execution import RouteResult,RouteProgress
    base=navigation_operation(robot,task)
    def result(identifier,value):
        if len(value.get('arrivals',()))!=expected_count:
            raise CompletionUnverified('route result does not cover every requested waypoint')
        return RouteResult(operation_id=identifier,arrivals=tuple(base._result_factory(identifier,item) for item in value['arrivals']))
    def progress(task):
        if not task.data:return None
        return RouteProgress(waypoint_index=task.data['waypoint_index'],waypoint_count=task.data['waypoint_count'],navigation=base._progress_factory(task))
    return Operation(task,result,progress,resource='mobility',cleanup_timeout=robot._welcome.features['operation_cleanup_timeout'])


def tool_operation(robot,task):
    from ..models.execution import ToolResult,ToolProgress
    def result(identifier,value):
        try:
            if abs(value['residual'])>value['tolerance']:raise ValueError('tool endpoint outside tolerance')
            return ToolResult(operation_id=identifier,stop=stop_report('tool:'+value['group'],value['stop_verification']),
                **{key:value[key] for key in ('group','joint','position','unit','target','residual','tolerance')})
        except (KeyError,ValueError,TypeError) as exc:raise CompletionUnverified('tool result lacks verified evidence') from exc
    return Operation(task,result,lambda task:None,resource='tool',cleanup_timeout=robot._welcome.features['operation_cleanup_timeout'])
