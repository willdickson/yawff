#!/usr/bin/env python
import scipy
import pylab
import libyawff

T = 6.0   # Period (s)
N = 10    # Number of stroke cycles
A_s = 70.0  # Stroke amplitude
A_r = 65.0  # Rotation amplitude
dt = 1.0/3000.0

run_params = {
    'dt'                : dt, 
    'yaw_inertia'       : 3.0,
    'yaw_damping'       : 0.000002,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 1.0,
    'yaw_filt_cut'      : 3.0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 100, 
    'integ_type'        : libyawff.INTEG_RKUTTA,
}

yawff = libyawff.Yawff(run_params)
num_motors = yawff.num_motors()

t = scipy.arange(0.0, T*N/dt)*dt
kine = scipy.zeros((t.shape[0],num_motors))
s0 = yawff.get_motor_num('stroke_0')
s1 = yawff.get_motor_num('stroke_1')
r0 = yawff.get_motor_num('rotation_0')
r1 = yawff.get_motor_num('rotation_1')
kine[:,s0] = A_s*scipy.cos(2.0*scipy.pi*t/T) 
kine[:,s1] = A_s*scipy.cos(2.0*scipy.pi*t/T)
kine[:,r0] = A_r*scipy.cos(2.0*scipy.pi*t/T)
kine[:,r1] = A_r*scipy.cos(2.0*scipy.pi*t/T) 
yawff.run(kine)

