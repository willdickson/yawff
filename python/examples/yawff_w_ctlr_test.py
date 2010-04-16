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
    'yaw_torq_deadband' : 0.0,
    'yaw_filt_lpcut'    : 10.0,
    'yaw_filt_hpcut'    : 0.0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 100, 
    'integ_type'        : libyawff.INTEG_RKUTTA,
    'startup_t'         : 0.0,
    'ff_flag'           : libyawff.FF_ON,
    'ctlr_flag'         : libyawff.CTLR_ON,
    'ctlr_type'         : libyawff.CTLR_TYPE_VEL,
    'ctlr_pgain'        : 1.0,
    'ctlr_dgain'        : 0.0,
    'kine_type'         : 'diff_aoa',
    'kine_period'       : 6.0,
    'stroke_amp'        : 90.0,
    'rotation_amp'      : 45.0,
    'stroke_k'          : 0.01,
    'rotation_k'        : 1.5,
}


yawff = libyawff.Yawff_w_Ctlr(run_params, pause_t=1.0)

T = 4.0*yawff.config_dict['kine_period'] 
N = int(T/dt) 
tt = scipy.arange(0.0,N)*dt
setpt = 5.0*scipy.sin(2.0*scipy.pi*tt/(2.0*yawff.config_dict['kine_period']))
setpt = setpt.reshape((N,1))
t, pos, vel, torq_flt, torq_raw, kine_dict, u =  yawff.run(setpt=setpt)

pylab.figure(1)
pylab.subplot(311)
pylab.title('wing kinematics')
pylab.plot(t,kine_dict['stroke_0'],'b')
pylab.plot(t,kine_dict['stroke_1'],'r')
pylab.ylabel('stroke')

pylab.subplot(312)
pylab.plot(t,kine_dict['rotation_0'],'b')
pylab.plot(t,kine_dict['rotation_1'],'r')
pylab.ylabel('rotation')

pylab.subplot(313)
pylab.plot(t,kine_dict['deviation_0'],'b')
pylab.plot(t,kine_dict['deviation_1'],'r')
pylab.ylabel('deviation')
pylab.xlabel('time (s)')

pylab.figure(2)
pylab.subplot(211)
pylab.plot(t,pos,'b')
pylab.plot(t,setpt,'r')
pylab.subplot(212)
pylab.plot(t,u)
pylab.xlabel('time (s)')
pylab.ylabel('u')
pylab.show()



