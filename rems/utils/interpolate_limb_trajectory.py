__author__ = "Yuki Shirai, Xuan Lin"
__email__ = "yukishirai4869@g.ucla.edu"
__copyright__ = "Copyright 2021 RoMeLa"
__date__ = "October 30, 2021"

__version__ = "0.2.0"
__status__ = "Prototype"

from geomdl import BSpline
from geomdl import utilities
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


# TODO: Make this function not robot dependent


class interpolation_method():  # Interpolation method

    def __init__(self):
        pass

    def interpolate(self, stepsBtwFrame, currentPos, nextPos, legNo):
        # This function returns a 18xstepsBtwFrame list that contains the interpolated frames for runmotorNextPos to run
        frameLength = []
        for i in legNo:
            frameLength.append(type.motor123((nextPos[i].coxa - currentPos[i].coxa) / stepsBtwFrame,
                                             (nextPos[i].femur - currentPos[i].femur) / stepsBtwFrame,
                                             (nextPos[i].tibia - currentPos[i].tibia) / stepsBtwFrame))

        interP = []

        for fr in range(stepsBtwFrame):
            oneInterP = []
            for i in legNo:
                oneInterP.append(type.motor123(currentPos[i].coxa + (fr + 1) * frameLength[i].coxa,
                                               currentPos[i].femur + (fr + 1) * frameLength[i].femur,
                                               currentPos[i].tibia + (fr + 1) * frameLength[i].tibia))
            interP.append(oneInterP[:])  # will cause pointer issue?

        # print("------------------------------ one interpolation -----------------------------")
        # print(stepsBtwFrame)
        # print(interP[0][0].coxa)
        # print(interP[1][0].coxa)
        # print(interP[2][0].coxa)
        # print(" ")
        return interP

    def interpolate_Linear(self, stepsBtwFrame, currentPos, nextPos, doInterpolation):
        """ generate an interpolation trajectory

        interpolate the given start and goal value lineary

        Args:
            stepsBtwFrame: number of interpolation steps between the currentPos and nextPos

            currentPos: start value of interpolation trajectory

            nextPos: goal value of interpolation trajectory

            doInterpolation: flag to tell if we actually do interpolation or not

        Returns:
            pts: generated interpolated trajectory

        """

        if doInterpolation == 0:
            return [nextPos] * (stepsBtwFrame)

        else:
            dim = len(nextPos)
            # create empty list
            pts = [[0.0 for dd in range(dim)] for ii in range(stepsBtwFrame)]
            # frameLength: dx between start and goal over the number of interpolation points (i.e., frame).
            frameLength = [(nextPos[dd] - currentPos[dd]) / stepsBtwFrame for dd in range(dim)]

            for fr in range(stepsBtwFrame):
                # for each frame, we generate interpolation for all dimension. we do the following calculation:
                # pts = (current) + k * dx
                pts[fr] = [currentPos[dd] + (fr + 1) * frameLength[dd] for dd in range(dim)]

            return pts

    def interpolate_Slerp(self, stepsBtwFrame, currentPos, nextPos, doInterpolation):
        """ generate an interpolation trajectory

        interpolate the given start and goal value using Spherical linear interpolation

        Args:
            stepsBtwFrame: number of interpolation steps between the currentPos and nextPos

            currentPos: start value of interpolation trajectory specified in quaternion

            nextPos: goal value of interpolation trajectory specified in quaternion

            doInterpolation: flag to tell if we actually do interpolation or not

        Returns:
            pts: generated interpolated trajectory

        reference:
        https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html#scipy.spatial.transform.Slerp
        """

        if doInterpolation == 0:
            return [nextPos] * (stepsBtwFrame)

        else:
            dim = len(nextPos)
            # # create empty list
            # pts = [[0.0 for dd in range(dim)] for ii in range(stepsBtwFrame)]
            # frameLength: dx between start and goal over the number of interpolation points (i.e., frame).
            key_times = [0, 1]
            dx = (1 - 0) / stepsBtwFrame
            frameLength = [((1 + dd) * dx) for dd in
                           range(stepsBtwFrame)]  # compute the ratio between the two quaternions

            key_rots = R.from_quat((currentPos, nextPos))

            slerp = Slerp(key_times, key_rots)
            interp_rots = slerp(frameLength)

            # to deal with the [w, x, y, z] quaternion in legIK
            quat = interp_rots.as_quat()[:, 1:]
            pts = np.append(quat, interp_rots.as_quat()[:, 0].reshape(2, 1), axis=1)

            return pts

    def interpolate_Linear_Triangle_Lift(self, stepsBtwFrame_up, stepsBtwFrame_down, currentPos, nextPos, peakPos):

        delta_x_up = (peakPos[0] - currentPos[0]) / stepsBtwFrame_up
        delta_y_up = (peakPos[1] - currentPos[1]) / stepsBtwFrame_up
        delta_z_up = (peakPos[2] - currentPos[2]) / stepsBtwFrame_up

        delta_x_down = (nextPos[0] - peakPos[0]) / stepsBtwFrame_down
        delta_y_down = (nextPos[1] - peakPos[1]) / stepsBtwFrame_down
        delta_z_down = (nextPos[2] - peakPos[2]) / stepsBtwFrame_down

        pts = [[0.0, 0.0, 0.0] for ii in range(stepsBtwFrame_up + stepsBtwFrame_down)]

        for iter_pts in range(stepsBtwFrame_up):  # Up part
            pts[iter_pts] = [currentPos[0] + (iter_pts + 1) * delta_x_up,
                             currentPos[1] + (iter_pts + 1) * delta_y_up,
                             currentPos[2] + (iter_pts + 1) * delta_z_up]

        for iter_pts in range(stepsBtwFrame_down):  # Down part
            pts[stepsBtwFrame_up + iter_pts] = \
                [peakPos[0] + (iter_pts + 1) * delta_x_down,
                 peakPos[1] + (iter_pts + 1) * delta_y_down,
                 peakPos[2] + (iter_pts + 1) * delta_z_down]

        return pts

    def interpolate_Bspline(self, stepsBtwFrame, ctrlPoint, currentPos, nextPos, doInterpolation, draw):
        # This function returns a list of 3-D trajectory points. Need to use IK to convert to joint angles afterwards
        # The return looks like [[0,0,0], [0,0,0], [0,0,0], [0,0,0], ... [0,0,0]] length=stepsBtwFrame
        # next = [next_dynamicToe[whichLeg].x, next_dynamicToe[whichLeg].y, next_dynamicToe[whichLeg].z]
        # curr = [current_dynamicToe[whichLeg].x, current_dynamicToe[whichLeg].y, current_dynamicToe[whichLeg].z]

        if doInterpolation == 0:
            return [nextPos] * (stepsBtwFrame)

        else:

            nurbs = BSpline.Curve()

            # Set unweighted control points

            allPts = [currentPos]
            for i in range(len(ctrlPoint)):
                allPts.append(ctrlPoint[i])

            allPts.append(nextPos)

            nurbs.degree = 2  # Degree of the curve, order = degree+1

            nurbs.ctrlpts = allPts

            # Auto-generate knot vector
            nurbs.knotvector = utilities.generate_knot_vector(nurbs.degree,
                                                              len(nurbs.ctrlpts))  # use the stuff from swf program?

            # depending on how long the looping time is and how smooth you want the trajectory to be,
            # this value should be carefully tuned
            nurbs.sample_size = stepsBtwFrame + 1  # The total # of points that goes to pts = nurbs._curve_points

            nurbs.evaluate()  # need to put start and stop points

            pts = nurbs.evalpts[1:(stepsBtwFrame + 1)]

            if draw == 1:
                # ==================== For climbing between 2 walls ====================
                plt.plot([nurbs.evalpts[i][0] for i in range(nurbs.sample_size)],
                         [nurbs.evalpts[i][2] for i in range(nurbs.sample_size)])
                plt.xlabel("x")
                plt.ylabel("z")
                plt.show(block=True)

                ##################### For normal walking #####################
                # plt.plot([nurbs._curve_points[i][1] for i in range(nurbs.sample_size)], [nurbs._curve_points[i][2] for i in range(nurbs.sample_size)])
                # plt.xlabel("y")
                # plt.ylabel("z")
                # plt.xlim([-130, 130])
                # plt.ylim([-5, 80])
                # plt.show(block=True)

            return pts

    def interpolate_Cycloid(self, stepsBtwFrame, currentPos, nextPos):

        assert currentPos[2] == nextPos[
            2], "Error interpolation: Cycloid trajectory starting and ending position should have identical height"

        # In one period, cycloid steps forward by 2*pi*r, calculate r from forward amount

        forward = np.sqrt((nextPos[0] - currentPos[0]) * (nextPos[0] - currentPos[0]) + (nextPos[1] - currentPos[1]) * (
                nextPos[1] - currentPos[1]))
        radius = forward / (2 * np.pi)

        t = np.linspace(0, 2 * np.pi, stepsBtwFrame)

        d = radius * (t - np.sin(t))
        z = radius * (1 - np.cos(t))
        theta = np.arctan2(nextPos[1] - currentPos[1], nextPos[0] - currentPos[0])

        pts = [[currentPos[0] + d[ii] * np.cos(theta),
                currentPos[1] + d[ii] * np.sin(theta),
                currentPos[2] + z[ii]] for ii in range(stepsBtwFrame)]  # Row: stepsBtwFrame, Column: 3

        return pts

    def interpolate_Cycloid_new(self, stepsBtwFrame, currentPos, nextPos, apex_height):
        pts = []
        t = np.linspace(0, 2 * np.pi, stepsBtwFrame)

        for i in range(stepsBtwFrame):
            t_curr = t[i]
            # calculate new x
            x_new = self.cycloid_function_xy(t_curr, currentPos[0], nextPos[0])
            # calculate new y
            y_new = self.cycloid_function_xy(t_curr, currentPos[1], nextPos[1])
            # calculate new z
            z_new = self.cycloid_function_z(t_curr, currentPos[2], nextPos[2], apex_height)
            new_pos = [x_new, y_new, z_new]
            pts.append(new_pos)

        return pts

    def cycloid_function_z(self, t, p_initial, p_final, apex_height):
        s = t

        if s < np.pi:
                new_z_pos = p_initial + max((p_final - p_initial) + apex_height, apex_height) * (1 - np.cos(s)) / 2
        else:
                pi_ipi = p_initial + max((p_final - p_initial) + apex_height, apex_height) * (1 - np.cos(np.pi)) / 2
                new_z_pos = p_final + (pi_ipi - p_final) * (1 - np.cos(s)) / 2

        return new_z_pos

    def cycloid_function_xy(self, t, pi, pf):
        s = t
        new_x_pos = pi + (pf - pi) * (s - np.sin(s)) / (2 * np.pi)

        return new_x_pos


if __name__ == '__main__':
    inter = interpolation_method()
    curr = [300,400,0.0]
    next_pos = [300,600,-20.0]
    ret = inter.interpolate_Cycloid_new(10,curr,next_pos,20)
    z_vec = []

    for i in range(len(ret)):
        curr_pos = ret[i]
        z_vec.append(curr_pos[2])

    plt.plot(z_vec)
    plt.show()

