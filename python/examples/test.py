#!/usr/bin/env python
import scipy 
import libyawff

N=300000

config = {
    'dev_name'       : '/dev/comedi0',
    'ain_subdev'     :  0,
    'dio_subdev'     :  2,
    'num_motor'      : 7,
    'yaw_motor'      : 6,
    'dio_clk'        : (0,2,4,6,8,10,12),
    'dio_dir'        : (1,3,5,7,9,11,13),
    'kine_map'       : (0,1,2,3,4,5),
    'dio_disable'    : 23,
    'yaw_ain'        : 0,
    'yaw_volt2torq'  : 5.0,
    'yaw_inertia'    : 0.00005,
    'yaw_ind2deg'    : 360.0/2000.0,
    'yaw_torq_lim'   : 0.5,
    'yaw_filt_cut'   : 1.0,
    'yaw_damping'    : 0.00001,
    'dt'             : 1.0/3000.0,
}


kine = scipy.zeros((N,config['num_motor']))

t, pos, vel, torq, end_pos = libyawff.yawff(kine, config)
