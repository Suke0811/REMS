

class RunConfig:
    def __init__(self, DT=None, realtime=False, to_thread=True):
        self.DT = DT
        self.realtime = realtime
        self.to_thread = to_thread
        self.block = True
