#!/usr/bin/env python
import scipy 
import pylab
import libyawff

N=100000

config = {
    'dev_name'          : '/dev/comedi0',
    'ain_subdev'        : 0,
    'dio_subdev'        : 2,
    'num_motor'         : 7,
    'yaw_motor'         : 6,
    'dio_clk'           : (0,2,4,6,8,10,12),
    'dio_dir'           : (1,3,5,7,9,11,13),
    'kine_map'          : (0,1,2,3,4,5),
    'dio_disable'       : 23,
    'yaw_ain'           : 0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 500, 
    'yaw_volt2torq'     : 0.05,
    'yaw_inertia'       : 0.0008,
    'yaw_ind2deg'       : 180.0/2000.0,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 1.0,
    'yaw_filt_cut'      : 10.0,
    'yaw_damping'       : 0.000002,
    'dt'                : 1.0/3000.0,
    'integ_type'        : libyawff.INTEG_RKUTTA,
}

kine = scipy.zeros((N,config['num_motor']))

t, pos, vel, torq, end_pos = libyawff.yawff(kine, config)

print torq.shape

pylab.subplot(311)
pylab.plot(t,pos)
pylab.ylabel('positin (deg)')

pylab.subplot(312)
pylab.plot(t,vel)
pylab.ylabel('velocity (deg/s)')

pylab.subplot(313)
pylab.plot(t,torq[:,1], 'b')
pylab.plot(t,torq[:,0], 'r')
pylab.xlabel('time (s)')
pylab.ylabel('torque (Nm)')

pylab.show()
