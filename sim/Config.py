INDEFINITE = -1.0

class SimConfig:
    def __init__(self, max_duration, dt=0.1, realtime=True, start_time=0, run_speed=1.0):
        self.dt = dt
        self.max_duration = max_duration
        self.realtime = realtime
        self.start_time = start_time
        self.run_speed = run_speed

