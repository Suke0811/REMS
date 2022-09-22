from sim.inputs import InputBase
import logging
import pygame
from sim.inputs.map.JOYSTICK_KEYMAP import *
from sim.typing import DefDict


class JoystickInput(InputBase):
    def __init__(self, stick_id=None, stick_name=None):
        super().__init__()
        self._inpts = {}
        # for pygame
        self._joysticks = {}
        if stick_id is None and stick_name is None:
            stick_id = 0
        self.stick_id = stick_id
        self.stick_name = stick_name
        self.stick_names = []
        self.buttons = {}
        self.axes = {}
        self.axes_deadzone = {}


    def init(self, input_def=None):
        super().init()
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        ('Number of Joysticks detected: ' + str(count))

        if count <= 0:
            raise ModuleNotFoundError('No Joystick is detected')
        for i in range(count):
            try:
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                name = joystick.get_instance_id()
                self.stick_names.append(name)
                self._joysticks[name] = joystick
                MAP: JOYSTIC_BASE = SUPPORTED_JOYSTICKs.get(joystick.get_name())
                if MAP is None:
                    MAP: JOYSTIC_BASE = SUPPORTED_JOYSTICKs.get(DEFAULT)

                self.buttons[name] = DefDict(MAP.button, dtype=bool)
                self.axes[name] = DefDict(MAP.axis)
                self.axes_deadzone[name] = DefDict(MAP.axis).set(MAP.axis_deadzone)

                logging.info("Joystick name: {}".format(self.stick_names))
            except ConnectionError:
                logging.error("Connection Failed")
                self.close = True
        if self.stick_id > i:
            raise ModuleNotFoundError(f'Joystick ID {self.stick_id} was not found. Available id: 0-{i}')

    def __del__(self):
        try:
            pygame.quit()
        except AttributeError:
            pass

    def get_inputs(self, timestamp=None, prefix='inpt', *args, **kwargs):
        if not self.stick_names:
            self.init()
        self._capture_joystick()
        if self.stick_id is not None:
            main_stick = self.stick_names[self.stick_id]
        else:
            main_stick = self.stick_name
        return {**self.axes[main_stick], **self.buttons[main_stick]}


    def if_exit(self):
        return self._quit

# internal
    def _capture_joystick(self):
        pygame.event.pump()
        for name, joystick in self._joysticks.items():
            axes = []
            deadzone = self.axes_deadzone[name].list()
            buttons = []
            # capture joystick values
            num_axes = joystick.get_numaxes()
            for i in range(min(num_axes, len(deadzone))):
                axes.append(self._filter_stick(joystick.get_axis(i), deadzone[i]))

            num_button = joystick.get_numbuttons()
            for i in range(num_button):
                buttons.append(joystick.get_button(i))

            self.axes[name].set(axes)
            self.buttons[name].set(buttons)

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
        print(j.get_inputs())
        #print(j.axes[j.stick_name[0]].data)
        #print(j.buttons[j.stick_name[0]].data)
        time.sleep(1)
