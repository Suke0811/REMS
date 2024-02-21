try:
    from .KeyboardInput import KeyboardInput
    from .KEYBOARD_KEYMAP import *
except ImportError:
    print("Keyboard Input requires 'pip install rems[keyboard]")

