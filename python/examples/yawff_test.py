#!/usr/bin/env python
import scipy
import pylab
import libyawff
import cPickle as pickle

T = 6.0   # Period (s)
N = 5    # Number of stroke cycles
amp_stroke = 70.0    # Stroke amplitude
amp_rotation = 45.0  # Rotation amplitude
dt = 1.0/3000.0
f = 1.0/T
datafile = 'test_data.pkl'
ds_n = 10 # Resampling step

run_params = {
    'dt'                : dt, 
    'yaw_inertia'       : 3.22,
    'yaw_damping'       : 0.0,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 0.0,
    'yaw_filt_lpcut'    : 10.0,
    'yaw_filt_hpcut'    : 0.0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 500, 
    'integ_type'        : libyawff.INTEG_RKUTTA,
    'startup_t'         : 0.0,
    'ff_flag'           : libyawff.FF_ON,
}

yawff = libyawff.Yawff(run_params)

t = scipy.arange(0.0, T*N/dt)*dt

if 0:
    u = libyawff.control_step(t, 0.0, 5.0, 4*T, 5*T, 12*T, 13*T)
    yawff.set_stroke_tilt_kine(t, f, u, amp_stroke, amp_rotation, rotation_offset=0.0)
if 0:
    u = libyawff.control_step(t, 0.0, 10.0, 3*T, 3.25*T, 11*T, 11.25*T)
    yawff.set_diff_aoa_kine(t, f, u, amp_stroke, amp_rotation, rotation_offset=2.0)
if 1:
    yawff.set_zero_kine(45.0)

if 0:
    pylab.plot(t,u)
    pylab.show()
if 0:
    yawff.plot_kine()
   
t, pos, vel, torq_flt, torq_raw =  yawff.run()

# Save data to file
data = {'t'         : libyawff.resample(t,ds_n),
        'pos'       : libyawff.resample(pos,ds_n),
        'vel'       : libyawff.resample(vel,ds_n),
        'torq_flt'  : libyawff.resample(torq_flt,ds_n),
        'torq_raw'  : libyawff.resample(torq_raw,ds_n),
        'config'    : yawff.config_dict,
        'kine'      : libyawff.resample(yawff.kine_deg,ds_n),}
try:
    data['u'] = libyawff.resample(u,ds_n)
except:
    pass

fd = open(datafile,'wb')
pickle.dump(data,fd)
fd.close()

pylab.subplot(411)
pylab.plot(data['t'],data['pos'])
pylab.ylabel('positin (deg)')

pylab.subplot(412)
pylab.plot(data['t'],data['vel'])
pylab.ylabel('velocity (deg/s)')

pylab.subplot(413)
pylab.plot(data['t'],data['torq_raw'], 'b')
pylab.plot(data['t'],data['torq_flt'], 'r')
pylab.ylabel('torque (Nm)')

try:
    pylab.subplot(414)
    pylab.plot(data['t'],data['u'])
    pylab.ylabel('ctl')
    pylab.xlabel('time (s)')
except:
    pass

pylab.show()

