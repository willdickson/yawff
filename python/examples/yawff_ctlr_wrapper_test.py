#!/usr/bin/env python 
import libmove_motor 
import scipy 
import pylab
import libyawff

RAD2DEG = 180.0/scipy.pi

# Read in data from motor map
mapfile = 'yawff_motor_maps.conf'
motor_maps = libmove_motor.read_motor_maps(mapfile)
clk_pins, dir_pins = libmove_motor.get_clkdir_pins(motor_maps)
yaw_num = motor_maps['yaw']['number']
yaw_ind2deg = motor_maps['yaw']['deg_per_ind']
motor_num_list = libmove_motor.get_motor_num_list(motor_maps)
num_motor = len(motor_num_list)
kine_map = tuple([i for i in motor_num_list if i != yaw_num]) 
motor_name_map = libmove_motor.get_num2name_map(motor_maps)
motor_cal = libmove_motor.get_motor_cal(motor_maps)

config = {
    'dev_name'          : '/dev/comedi0',
    'ain_subdev'        : 0,
    'dio_subdev'        : 2,
    'num_motor'         : num_motor,
    'yaw_motor'         : yaw_num,
    'dio_clk'           : clk_pins,
    'dio_dir'           : dir_pins,
    'kine_map'          : kine_map,
    'motor_name_map'    : motor_name_map,
    'dio_disable'       : 23,
    'yaw_ain'           : 0,
    'yaw_ain_zero_dt'   : 0.01,
    'yaw_ain_zero_num'  : 100, 
    'yaw_volt2torq'     : 0.01,
    'yaw_inertia'       : 3.0,
    'yaw_ind2deg'       : yaw_ind2deg,
    'yaw_torq_lim'      : 0.5,
    'yaw_torq_deadband' : 0.0,
    'yaw_filt_lpcut'    : 10.0,
    'yaw_filt_hpcut'    : 0.0,
    'yaw_damping'       : 0.000002,
    'dt'                : 1.0/3000.0,
    'integ_type'        : libyawff.INTEG_RKUTTA,
    'startup_t'         : 0.0,
    'ff_flag'           : libyawff.FF_ON,
    'ctlr_flag'         : libyawff.CTLR_ON,
    'ctlr_type'         : libyawff.CTLR_TYPE_VEL,
    'ctlr_pgain'        : 1.0,
    'ctlr_dgain'        : 0.3,
    'kine_type'         : 'diff_aoa',
    'kine_period'       : 6.0,
    'stroke_amp'        : 90.0,
    'rotation_amp'      : 45.0,
    'stroke_k'          : 0.01,
    'rotation_k'        : 1.5,
    'motor_cal'         : motor_cal,
}


num_cycle = 1
N = int(num_cycle*config['kine_period']/config['dt'])
print 'N = %d'%(N,)

tt = config['dt']*scipy.arange(0,N)
tt = tt.reshape((N,1))
setpt = 10.0*scipy.sin(2.0*scipy.pi*tt/(2.0*config['kine_period']))
#setpt = 5.0*scipy.ones((N,1))

#setpt = scipy.zeros((N,1))

t, pos, vel, torq, kine, u, end_pos = libyawff.yawff_ctlr_c_wrapper(setpt, config)

pos = RAD2DEG*pos
vel = RAD2DEG*vel

print 
print 't.shape', t.shape
print 'pos.shape', pos.shape
print 'vel.shape', vel.shape
print 'torq.shape', torq.shape
print 'kine.shape', kine.shape
print 'u.shape', u.shape

for i in range(0,kine.shape[1]):
    if config['motor_name_map'][i] in ('rotation_0', 'rotation_1'):
        pylab.plot(t,kine[:,i])
        print kine[:,i].max(), kine[:,i].min()
pylab.show()

#pylab.plot(tt,setpt,'r')
#pylab.plot(tt,dsetpt,'b')
#pylab.plot(t, (180.0/scipy.pi)*u, 'g')
#pylab.show()

#pylab.subplot(311)
#pylab.plot(t,pos)
#pylab.ylabel('positin (deg)')
#
#pylab.subplot(312)
#pylab.plot(t,vel)
#pylab.ylabel('velocity (deg/s)')
#
#pylab.subplot(313)
#pylab.plot(t,torq[:,1], 'b')
#pylab.plot(t,torq[:,0], 'r')
#pylab.xlabel('time (s)')
#pylab.ylabel('torque (Nm)')
#
#pylab.show()
