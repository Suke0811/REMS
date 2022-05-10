from sim.constants import ROBOT, ENVIRONMENT, TEST
from sim.formulation import INPUT_SPACE, OUTPUT_SPACE
import numpy as np
# TODO: ADD FUNCTION TO TURN OF UPDATE OF TUNER FOR SOME OF THE COMPONENTS (THROUGH H cost)
class DifferentialDrive:
    def __init__(self):
        """init with a specific initial state (optional) """
        self.LENGTH = ROBOT.LENGTH
        self.WIDTH = ROBOT.WIDTH
        self.WHEEL_RADIUS = ROBOT.WHEEL_RADIUS
        self.FRONT_LIDAR_POS = ROBOT.FRONT_LIDAR_POS
        self.RIGHT_LIDAR_POS = ROBOT.RIGHT_LIDAR_POS
        self.SERVO_SPEED = ROBOT.SERVO_SPEED
        self.PARAMS = ROBOT.PARAMS
        self.NN_OUTPUT = ROBOT.NN_OUTPUT
        self.dt = TEST.DT

    def next_state(self,cur_state, inpts):
        """inpts is a 2x1 array, first index is the left wheel [-1,1], and second index is the right wheel [-1,1].
        cur_state is a 3x1 array, first index is x position, second index is y position, third index is theta at the
        current time step. params is a 12x1 array, which come from the auto-tuner, they represent the weights of cubic polynomials for x,y, and th
        :return next state after propagation from time step t to t+1 by self.DT
        """
        th_t = cur_state[2]
        self.B = np.array([[(self.WHEEL_RADIUS/2)*self.SERVO_SPEED*np.cos(th_t),
                           (self.WHEEL_RADIUS/2)*self.SERVO_SPEED*np.cos(th_t)],
                          [(self.WHEEL_RADIUS/2)*self.SERVO_SPEED*np.sin(th_t),
                          (self.WHEEL_RADIUS/2)*self.SERVO_SPEED*np.sin(th_t)],
                          [(-self.WHEEL_RADIUS/self.LENGTH)*self.SERVO_SPEED,(self.WHEEL_RADIUS/self.LENGTH)*self.SERVO_SPEED]])

        self.W = np.zeros((3,))

        W_x = self.NN_OUTPUT[0]
        W_y = self.NN_OUTPUT[1]
        W_dth = self.NN_OUTPUT[2]

        self.W[0] = W_x
        self.W[1] = W_y
        self.W[2] = W_dth

        cur_state = np.array(cur_state)

        next_state = cur_state[0:3] + np.matmul(self.B,inpts)*self.dt
        next_state[0] = next_state[0]+self.W[0]*self.dt
        next_state[1] = next_state[1]+self.W[1]*self.dt
        dth = (self.WHEEL_RADIUS/self.LENGTH)*self.SERVO_SPEED*(inpts[1]-inpts[0])

        next_state = np.append(next_state,dth)
        next_state[3] = next_state[3] + self.W[2]*self.dt
        cur_state[2] = self.bound_rad(cur_state[2])
        next_state[2] = self.dt * next_state[3] + cur_state[2]
        next_state[2] = self.bound_rad(next_state[2])

        return next_state

    def bound_rad(self, rad):
        rad = self.wrap_to_pi([rad])[0]
        if 0.0 > rad and rad < -np.pi:
            rad_diff = rad + np.pi
            rad = np.pi+rad_diff
        elif 0.0 < rad and rad > np.pi:
            rad_diff = rad - np.pi
            rad = -np.pi + rad_diff

        return rad

    def wrap_to_pi(self, rad):
        """
        Wrap the angle (in radiants) to within [-pi, pi)
        :param rad:
        :return:
        """
        ret_rad = rad[:]

        for iter in range(len(ret_rad)):

            while ret_rad[iter] >= np.pi:
                ret_rad[iter] -= 2.0 * np.pi

            while ret_rad[iter] < -np.pi:
                ret_rad[iter] += 2.0 * np.pi

        return ret_rad
