#!/usr/bin/env python
import sys
import scipy
import pylab
import libyawff
import cPickle as pickle

motor_name = sys.argv[1]
move_ind = int(sys.argv[2])

dt = 1.0/5000.0

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
yawff.move_by_ind(motor_name, move_ind, vmax=50)

