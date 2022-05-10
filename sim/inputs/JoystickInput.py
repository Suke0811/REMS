from sim.inputs import InputBase
import logging
import pygame
from sim.inputs.binding.JOYSTICK_KEYMAP import *
from sim.type import DefDict


class JoystickInput(InputBase):
    def __init__(self, ):
        super().__init__()
        self._inpts = {}
        # for pygame
        self._joysticks = {}
        self.stick_name = []
        self.buttons = {}
        self.axes = {}
        self.axes_deadzone = {}

    def init(self):
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count <= 0:
            raise ModuleNotFoundError('No Joystick is detected')
        for i in range(count):
            try:
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                name = joystick.get_name()
                self.stick_name.append(name)
                self._joysticks[name] = joystick
                MAP: JOYSTIC_BASE = SUPPORTED_JOYSTICKs.get(name)
                if MAP is None:
                    MAP: JOYSTIC_BASE = SUPPORTED_JOYSTICKs.get(DEFAULT)

                self.buttons[name] = DefDict(MAP.button, type_=bool)
                self.axes[name] = DefDict(MAP.axis)
                self.axes_deadzone[name] = DefDict(MAP.axis_deadzone)

                logging.info("Joystick name: {}".format(self.stick_name))
            except ConnectionError:
                logging.error("Connection Failed")
                self.close = True

    def __del__(self):
        pygame.quit()

    def get_inputs(self, timestamp=None):
        if not self.stick_name:
            self.init()
        self._capture_joystick()
        return self._inpts

    def if_exit(self):
        return self._quit

# internal
    def _capture_joystick(self):
        pygame.event.pump()
        for name, joystick in self._joysticks.items():
            axes = []
            deadzone = self.axes_deadzone[name].data.as_list()
            buttons = []
            # capture joystick values
            num_axes = joystick.get_numaxes()
            for i in range(min(num_axes, len(deadzone))):
                axes.append(self._filter_stick(joystick.get_axis(i), deadzone[i]))

            num_button = joystick.get_numbuttons()
            for i in range(num_button):
                buttons.append(joystick.get_button(i))

            self.axes[name].data = axes
            self.buttons[name].data = buttons

    @staticmethod
    def _filter_stick(axis, deadzone):
        """analog stick needs to be filtered out for deadzone"""
        if -deadzone < axis < deadzone:
            axis = 0.0
        return axis

if __name__ == "__main__":
    import time
    j = JoystickInput()
    while True:
        j.get_inputs()
        print(j.axes[j.stick_name[0]].data)
        time.sleep(1)
