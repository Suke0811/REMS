from pynput.keyboard import Key
from sim.type import DefDict

KEYBOARD_MAP = [chr(i) for i in range(33, 126)]
KEYBOARD_MAP.extend([d.name for d in Key])

KEYBOARD_DEF = DefDict(KEYBOARD_MAP, bool())
