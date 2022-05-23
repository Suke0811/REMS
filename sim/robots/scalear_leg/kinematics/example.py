from SCALAR_kinematics import ScalerKinematics
import numpy as np
from time import perf_counter

my_scalar_k = ScalerKinematics()

joint_angles = [0.2,0.3,np.pi/2,-0.5,1.1, -1.8]
which_leg = 0

fk_res = my_scalar_k.scalar_forward_kinematics(which_leg, joint_angles)

#print(fk_res)


N = int(2e5)

st = perf_counter()
for _ in range(N):
    ik_res = my_scalar_k.scalar_inverse_kinematics(which_leg, fk_res)
et = perf_counter()-st

print(f"Joint Angel to FK: {joint_angles}")
print(f"Joint Angle Calculated from IK: {ik_res}")
print(f"Total Calculation time: {et} for {N} loop")
print(f"Time per Ik {et/N} s")
print(f"Time per Ik {et/N*1e3} ms")

