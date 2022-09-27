from rems.outputs import OutputBase
from matplotlib import pyplot as plt

class GridAnimation(OutputBase):
    def __init__(self, grid):
        super().__init__()
        self.grid = grid

    def init(self):
        self.grid.show(0, 0)
        plt.show(block=False)


    def process(self, state, inpt, outpt, timestamp, info):
        self.grid.show(*state.filter(['x', 'y']).list())

