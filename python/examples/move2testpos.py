#!/usr/bin/env python
import scipy
import pylab
import libyawff
import cPickle as pickle

dt = 1.0/3000.0

run_params = {
    'dt'                : dt, 
    'yaw_inertia'       : 3.22,
    'yaw_damping'       : 0.0,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 1.5,
    'yaw_filt_lpcut'    : 3.0,
    'yaw_filt_hpcut'    : 0.0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 500, 
    'integ_type'        : libyawff.INTEG_RKUTTA,
    'startup_t'         : 0.0,
    'ff_flag'           : libyawff.FF_ON,
}

yawff = libyawff.Yawff(run_params)
#yawff.move_to_test_pos('zero')
yawff.move_rot_to_pos(90,vmax=60)

