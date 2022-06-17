from sim.sim_handler.job_background.JobHandler import JobHandler
from concurrent.futures import ProcessPoolExecutor

import os


class JobHandlerMP(JobHandler):
    def __init__(self, main=None):
        """To make this to work, in the top level code, there should be
        if __name__ == '__main__': for windows and hand over
        __name__ instance to this class. Windows spawn a new process which
        restarts the main again"""
        super().__init__()
        self.main = main
        self.executor = ProcessPoolExecutor()
        if self.main is None and os.name == 'nt':
            raise OSError('Wiondows requires __name__ from the top level main() and safe guard if __name__ == \'__main__\'')

    def in_main(self):
        if self.main is None or self.main == '__main__':
            return True
