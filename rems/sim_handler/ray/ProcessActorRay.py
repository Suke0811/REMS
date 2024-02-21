import ray
from rems.sim_handler.ray.RobotRayWrapper import RobotRayWrapper
from rems.sim_handler.ProcessActor import ProcessActor
@ray.remote
class ProcessActorRay(ProcessActor):
    def __init__(self, process, *args, **kwargs):
        for a in args:
            if isinstance(a, RobotRayWrapper):
                a._reset_attr()
        super().__init__(process, *args, **kwargs)
