from sim.inputs import InputBase
from pynput.keyboard import Key, Listener
from sim.type import DefDict
from sim.inputs.map.KEYBOARD_KEYMAP import KEYBOARD_DEF
import numpy as np

class KeyboardInput(InputBase):
    def __init__(self):
        super().__init__()
        self._keys = KEYBOARD_DEF
        self._start_capture_key()   # set listener

    def get_inputs(self, inpt_def: DefDict, timestamp = None):
        inpt_def = self._inputs
        return inpt_def

    def if_exit(self):
        return self._quit

    def _start_capture_key(self):
        # listener for press/release events
        def on_press(key):
            if key == Key.esc:
                self._done = True
                return False    # kill listener
            try:
                k = key.char
            except AttributeError:
                k = key.name

            self._keys.data = {k: True}
            pass

        def on_release(key):
            if key == Key.esc:
                self._done = True
                return False    # kill listener
            try:
                k = key.char
            except AttributeError:
                k = key.name

            self._keys.data = {k: False}

        listener = Listener(on_press=on_press, on_release=on_release)
        listener.start()


if __name__ == '__main__':
    import time
    k = KeyboardInput()
    while True:
        k.get_inputs()
        print(any(k._keys.data.list()))
        time.sleep(1)
