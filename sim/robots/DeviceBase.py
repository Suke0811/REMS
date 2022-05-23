from sim.type import DefDict

class DeviceBase:
    def init(self):
        self.open()
        self.enable(True)

    def open(self):
        pass

    def close(self):
        pass

    def enable(self, enable):
        pass

    def drive(self, inpt: DefDict, timestamp):
        raise NotImplementedError

    def sense(self):
        raise NotImplementedError

    def observe_state(self):
        raise NotImplementedError

    def __del__(self):
        self.enable(False)
        self.close()
