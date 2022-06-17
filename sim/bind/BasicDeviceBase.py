

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

    def __del__(self):
        self.enable(False)
        self.close()
