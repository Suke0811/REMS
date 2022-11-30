from rems import DefDict, MapRule
from rems.typing.std.StdDefinitions import POS_3D, ROT_MAT

import numpy as np
import copy

class Tmat:
    def __init__(self, values, fr_from, fr_to):
        self.tmat = self.create_Tmat()
        self.fr_from = 'base'
        self.fr_tp = 'end'


    def update(self, values):
        if self.is_Tmat(values):
            pass

        if self.is_np_Tmat(values):
            pass





    def transpose(self):
        T = self.as_np()
        return np.transpose(T)

    def as_np(self):
        return self.tmat.as_ruled()


    @staticmethod
    def is_np_Tmat(values):
        is_valid = False
        if isinstance(values, np.ndarray):
            if values.shape == (4, 4):
                is_valid = True
        return is_valid

    @staticmethod
    def is_Tmat(values):
        return isinstance(values, Tmat)


    def create_Tmat(self):
        def T_mat_rule(r11, r12, r13,
                       r21, r22, r23,
                       r31, r32, r33,
                       x, y, z):
            return np.array([[r11, r12, r13, x],
                             [r21, r22, r23, y],
                             [r31, r32, r33, z],
                             [0, 0, 0, 1]])

        def Tmat2dict(row0, row1, row2):
            return dict(r11=row0[0], r12=row0[1], r13=row0[2],
                        r21=row1[0], r22=row1[1], r23=row1[2],
                        r31=row2[0], r32=row2[1], r33=row2[2],
                        x=row0[3], y=row1[3], z=row2[3])

        T_keys = list(ROT_MAT.keys()) + list(POS_3D.keys())
        return DefDict(definition=(ROT_MAT, POS_3D),
                    format_rule=MapRule(T_keys, func=T_mat_rule, inv_func=Tmat2dict, to_list=True))



    def __matmul__(self, other):
        current_np = self.as_np()
        other_np = None
        if self.is_Tmat(other):
            other_np = other.as_np()

        if self.is_np_Tmat(other):
                other_np = other

        if other_np is None:
            raise TypeError(f'{other} must be type of 4*4 numpy array or Tmat')

        return current_np @ other_np
