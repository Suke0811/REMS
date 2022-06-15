

class JOYSTIC_BASE:
    name = None
    button = []
    axis = []
    axis_deadzone = []
    hat = []
    ball = []

class X56THROTTLE(JOYSTIC_BASE):
    name ='Saitek Pro Flight X-56 Rhino Throttle'
    button = ['E',
              'F_Push', 'G_Push',
              'I', 'H',
              'SW1', 'SW2', 'SW3', 'SW4', 'SW5', 'SW6',
              'TGL1_Up', 'TGL1_Down', 'TGL2_Up', 'TGL2_Down', 'TGL3_Up', 'TGL3_Down', 'TGL4_Up', 'TGL4_Down',
              'H3_Up', 'H3_Right', 'H3_Down', 'H3_Left',
              'H4_Up', 'H4_Right', 'H4_Down', 'H4_Left',
              'K1_Up', 'K1_Down',
              'Scroll_FWD', 'Scroll_BCK',
              'Ministick_Click'
              'SLD',
              'M1', 'M2', 'S1'] # mode selection
    axis = ['Left_Throttle', 'Right_Throttle', 'Rotary_F', 'Ministick_X', 'Ministick_Y', 'Rotary_G', 'Rotary_3', 'Rotary_4']
    axis_deadzone = [0.05 for i in axis]



class X56Stick(JOYSTIC_BASE):
    name = 'Saitek Pro Flight X-56 Rhino Stick'
    button = ['Trigger', 'A', 'B', 'C_Stick', 'D',
              'Pinky',
              'H 1_Up', 'H1_Right', 'H1_Down', 'H1_Left',
              'H2_Up', 'H2_Right', 'H2_Down', 'H2_Left',
              'M1', 'M2', 'S1']
    axis = ['X', 'Y', 'C_Stick_X', 'C_Stick_Y', 'Twist']
    axis_deadzone = [0.05 for i in axis]
    hat = [('X', 'Y')]


class SwitchPro(JOYSTIC_BASE):
    name = 'Nintendo Switch Pro Controller'
    button = ['BUTTON_A', 'BUTTON_B', 'BUTTON_X', 'BUTTON_Y', 'BUTTON_LB', 'BUTTON_RB',]
    axis = ['STICK_LEFT_X', 'STICK_LEFT_Y', 'STICK_RIGHT_X', 'STICK_RIGHT_Y', 'THROTTLE_L', 'THROTTLE_R']
    axis_deadzone = [0.15, 0.15, 0.15, 0.15, 0.0, 0.0]


DEFAULT = 'Default'

SUPPORTED_JOYSTICKs = {X56THROTTLE.name: X56THROTTLE,
                       X56Stick.name: X56Stick,
                       SwitchPro.name: SwitchPro,
                       DEFAULT: SwitchPro}
