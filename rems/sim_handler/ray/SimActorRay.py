from rems.sim_handler.SimActor import SimActor
import ray

@ray.remote
class SimActorRay(SimActor):
    pass
