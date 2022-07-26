from scalar_sim import pyb_sim

URDF_PATH= '/home/max/ucla/scalar/AbstractedRobot/sim/robots/scaler/urdf_scalar_6DoF/urdf/SCALAR_6DoF.urdf'
my_sim = pyb_sim(urdf_filename=URDF_PATH, DoFnum=6, delta_t=0.01)
my_sim.dance()

