#!/usr/bin/env python
import scipy
import libyawff

N=100000
run_params = {
    'dt'                : 1.0/3000.0,
    'yaw_inertia'       : 3.0,
    'yaw_damping'       : 0.000002,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 1.0,
    'yaw_filt_cut'      : 3.0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 500, 
    'integ_type'        : libyawff.INTEG_RKUTTA,
}

yawff = libyawff.Yawff(run_params)
num_motors = yawff.num_motors()
kine = scipy.zeros((N,num_motors))
yawff.run(kine)

