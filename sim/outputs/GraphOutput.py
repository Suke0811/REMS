from sim.typing.DefDict import DefDict
from sim.outputs import OutputBase
class GraphOutput(OutputBase):
    def __init__(self):
        super().__init__()

    def process(self, state, inpt, outpt, timestamp, info):
        super().process(state, inpt, outpt, timestamp, info)
        """
        This part is called at every timestep
        """

    def make_output(self):
        """
        This will be called once after then run is done
        (Not necessary for now)
        """

    

