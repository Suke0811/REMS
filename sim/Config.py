INDEFINITE = -1.0

class SimConfig:
    def __init__(self, max_duration=INDEFINITE, dt=0.1, realtime=True, start_time=0, run_speed=None):
        self.dt = dt
        self.max_duration = max_duration
        self.realtime = realtime
        self.start_time = start_time
        if run_speed is None:
            self.run_speed = self.realtime
        else:
            self.run_speed = run_speed

    def if_time(self, t):
        if t <= self.max_duration+self.start_time:
            return False
        else:
            return True

