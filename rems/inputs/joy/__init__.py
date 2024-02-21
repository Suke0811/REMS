try:
    from .JoystickInput import JoystickInput
    from .JOYSTICK_KEYMAP import *
except ImportError:
    print("Keyboard Input requires 'pip install rems[keyboard]")
