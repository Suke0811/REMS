

class BasicDeviceBase:
    def init(self):
        self.open()
        self.enable(True)

    def open(self):
        pass

    def close(self):
        pass

    def enable(self, enable):
        pass

