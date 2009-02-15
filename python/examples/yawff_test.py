#!/usr/bin/env python
import scipy
import pylab
import libyawff
import cPickle as pickle

T = 6.0   # Period (s)
N = 30    # Number of stroke cycles
amp_stroke = 70.0    # Stroke amplitude
amp_rotation = 45.0  # Rotation amplitude
dt = 1.0/3000.0
f = 1.0/T
datafile = 'test_data.pkl'

run_params = {
    'dt'                : dt, 
    'yaw_inertia'       : 3.0,
    'yaw_damping'       : 0.0,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 1.5,
    'yaw_filt_cut'      : 3.0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 500, 
    'integ_type'        : libyawff.INTEG_RKUTTA,
}

yawff = libyawff.Yawff(run_params)

t = scipy.arange(0.0, T*N/dt)*dt
u = libyawff.control_step(t, 0.0, 5.0, 4*T, 5*T, 12*T, 13*T)

yawff.set_stroke_tilt_kine(t, f, u, amp_stroke, amp_rotation, rotation_offset=-1.0)
#yawff.plot_kine()
t, pos, vel, torq_flt, torq_raw =  yawff.run()

# Save data to file
data = {'t' : t,
        'pos' : pos,
        'vel' : vel,
        'torq_flt' : torq_flt,
        'torq_raw' : torq_raw,
        'config' : yawff.config_dict,
        'u' : u,
        'kine' : yawff.kine_deg}

fd = open(datafile,'wb')
pickle.dump(data,fd)
fd.close()

pylab.subplot(411)
pylab.plot(t,pos)
pylab.ylabel('positin (deg)')

pylab.subplot(412)
pylab.plot(t,vel)
pylab.ylabel('velocity (deg/s)')

pylab.subplot(413)
pylab.plot(t,torq_raw, 'b')
pylab.plot(t,torq_flt, 'r')
pylab.ylabel('torque (Nm)')

pylab.subplot(414)
pylab.plot(t,u)
pylab.ylabel('ctl')
pylab.xlabel('time (s)')
pylab.show()


