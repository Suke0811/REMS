from rems.inputs import InputBase
from pynput.keyboard import Key, Listener
from rems.typing import DefDict
from rems.inputs.map.KEYBOARD_KEYMAP import KEYBOARD_DEF
import numpy as np

class KeyboardInput(InputBase):
    def __init__(self, init_state=False, enable_keys=None):
        super().__init__()
        self._init_state = init_state
        if enable_keys is None:
            enable_keys = KEYBOARD_DEF.keys()
        self._enable_keys = enable_keys
        self._keys = KEYBOARD_DEF
        self._inputs = self._keys
        self._start_capture_key()   # set listener

    def get_inputs(self, timestamp=None, prefix='inpt', *args, **kwargs):
        if prefix == 'state':
            if self._init_state:
                return self._inputs
            else:
                return
        return self._inputs.filter(self._enable_keys)


    def if_exit(self):
        return self._quit

# this will capture press and release events
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

            self._keys.set({k: True})
            pass

        def on_release(key):
            if key == Key.esc:
                self._done = True
                return False    # kill listener
            try:
                k = key.char
            except AttributeError:
                k = key.name

            self._keys.set({k: False})
        # set listeners
        listener = Listener(on_press=on_press, on_release=on_release)
        listener.start()


if __name__ == '__main__':
    import time
    k = KeyboardInput()
    while True:
        print(k.get_inputs())
        time.sleep(1)
