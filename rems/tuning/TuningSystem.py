from rems.process import ProcessSystem

class TuningSystem(ProcessSystem):
    def __init__(self, target_robot, reference_robot):
        super().__init__()
        self.target_robot = target_robot
        self.ref_robot = reference_robot



