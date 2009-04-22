"""
-----------------------------------------------------------------------
yawff
Copyright (C) William Dickson, 2008.
  
wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of yawff.

yawff is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
    
yawff is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with yawff.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------
"""
import sys
import os
import os.path
import ConfigParser
import optparse
import libmove_motor
import libyawff
import scipy
import pylab
import scipy.interpolate
import time
import clkdirpwm

PI = scipy.pi
DEG2RAD = scipy.pi/180.0
RAD2DEG = 180.0/scipy.pi
BORFRC_DIR = os.path.join(os.environ['HOME'],'.borfrc')
DFLT_CONFIG_FILE = os.path.join(BORFRC_DIR, 'defaults')
DFLT_SENSOR_CAL_DIR = os.path.join(BORFRC_DIR,'sensor_cal')
DFLT_COMEDI_CONF_DIR = os.path.join(BORFRC_DIR, 'comedi_conf')
DFLT_MOVE_VMAX = 10.0
DFLT_MOVE_ACCEL = 40.0
DFLT_MOVE_DT = 1.0/3000.0
DFLT_PAUSE_T = 10.0

class Yawff:

    def __init__(self, 
                 run_params, 
                 motor_maps_file=None, 
                 sensor_cal_file=None, 
                 comedi_conf_file=None,
                 defaults_file=DFLT_CONFIG_FILE,
                 move_vmax=DFLT_MOVE_VMAX,
                 move_accel=DFLT_MOVE_ACCEL,
                 move_dt=DFLT_MOVE_DT):
        """
        Initialize Yawff class
        """
        if motor_maps_file==None or sensor_cal_file==None or comedi_conf_file==None:
            dflts = read_defaults(defaults_file)
            dflt_motor_maps_file, dflt_sensor_cal_file, dflt_comedi_conf_file = dflts

        # Read motor maps file
        if motor_maps_file == None:
            self.motor_maps_file = dflt_motor_maps_file
        else:
            self.motor_maps_file = motor_maps_file
        self.motor_maps = libmove_motor.read_motor_maps(self.motor_maps_file)

        # Read sensor calibration file
        if sensor_cal_file == None:
            self.sensor_cal_file = dflt_sensor_cal_file
        else:
            self.sensor_cal_file = sensor_cal_file
        self.sensor_cal = read_sensor_cal(self.sensor_cal_file)

        # Read comedi configuration file
        if comedi_conf_file == None:
            self.comedi_conf_file = dflt_comedi_conf_file
        else:
            self.comedi_conf_file = comedi_conf_file
        self.comedi_conf = read_comedi_conf(self.comedi_conf_file)

        self.run_params = run_params
        self.move_vmax = move_vmax
        self.move_accel = move_accel
        self.move_dt = move_dt
        self.config_dict = self.create_config_dict()
        self.kine_deg = None
        self.t = None
        self.pause_t = DFLT_PAUSE_T

    def create_config_dict(self):
        """
        Create configuration dictionary which is passed to yawff_c_wrapper
        """
        config = {}
        
        # Add comedi configuration parameters
        config.update(self.comedi_conf)

        # Add motor map parameters
        clk_pins, dir_pins = libmove_motor.get_clkdir_pins(self.motor_maps)
        yaw_motor = self.motor_maps['yaw']['number']
        yaw_ind2deg = self.motor_maps['yaw']['deg_per_ind']
        motor_num_list = libmove_motor.get_motor_num_list(self.motor_maps)
        num_motor = len(motor_num_list)
        kine_map = tuple([i for i in motor_num_list if i != yaw_motor]) 
        config['num_motor'] = num_motor
        config['yaw_motor'] = yaw_motor
        config['dio_clk'] = clk_pins
        config['dio_dir'] = dir_pins
        config['kine_map'] = kine_map
        config['yaw_ind2deg'] = yaw_ind2deg

        # Add sensor_calibration parameters
        config['yaw_volt2torq'] = self.sensor_cal['calibration']
        config['yaw_ain'] = self.sensor_cal['ain_chan']

        # Add run parameters
        config.update(self.run_params)
        return config

    #def move_to_test_pos(self, pos_name, vmax=DFLT_MOVE_VMAX, accel=DFLT_MOVE_ACCEL):
    #    """
    #    Move the robot into commonly used test positions
    #    """
    #    config = self.config_dict
    #    n = self.num_motors()
    #    if pos_name == 'zero':
    #        zero_pos = scipy.zeros((n,))
    #        self.move_to_pos(zero_pos,vmax=vmax,accel=accel) 
    #    elif pos_name == 'rot_plus_90':
    #        self.move_rot_to_pos(89.99,vmax=vmax,accel=accel)
    #    elif pos_name == 'rot_minus_90':
    #        self.move_rot_to_pos(-89.99,vmax=vmax,accel=accel)
    #    elif pos_name == 'yaw_90':
    #        pos = scipy.zeros((n,))
    #        yn = self.get_motor_num('yaw')
    #        pos[yn] = 90.0
    #        self.move_to_pos(pos,vmax=vmax,accel=accel) 
    #    elif pos_name == 'yaw_minus_90':
    #        pos = scipy.zeros((n,))
    #        yn = self.get_motor_num('yaw')
    #        pos[yn] = -90.0
    #        self.move_to_pos(pos,vmax=vmax,accel=accel) 
    #    elif pos_name == 'yaw_180':
    #        pos = scipy.zeros((n,))
    #        yn = self.get_motor_num('yaw')
    #        pos[yn] = 180.0
    #        self.move_to_pos(pos,vmax=vmax,accel=accel) 
    #    else:
    #        raise ValueError, 'unknown test position'

    #def move_rot_to_pos(self,ang, vmax=DFLT_MOVE_VMAX, accel=DFLT_MOVE_ACCEL):
    #    """
    #    Move robots rotation angles to the given position
    #    """
    #    n = self.num_motors()
    #    pos = scipy.zeros((n,))
    #    r0 = self.get_motor_num('rotation_0')
    #    r1 = self.get_motor_num('rotation_1')
    #    pos[r0] = ang 
    #    pos[r1] = ang
    #    self.move_to_pos(pos,vmax=vmax,accel=accel)

    def move_to_pos(self,name_list,pos_list,noreturn=False,vmax=DFLT_MOVE_VMAX, accel=DFLT_MOVE_ACCEL):
        """
        Move the robot to the given position.
        """
        n = self.num_motors()
        pos = scipy.zeros((n,))

        if len(pos_list) == len(name_list):
            for p, name in zip(pos_list,name_list):
                motor_num = self.get_motor_num(name)
                pos[motor_num] = p
        elif len(pos_list) == 1:
            p = pos_list[0]
            for name in name_list:
                motor_num = self.get_motor_num(name)
                pos[motor_num] = p
        else:
            raise ValueError, 'len(pos_list) must equal len(name_list) or 1'

        config = self.config_dict
        # Move to kinematics starting positon
        print 'moving to position'
        zero_indpos_deg = libmove_motor.get_zero_indpos_deg(self.motor_maps)
        ramps_deg = libmove_motor.get_ramp_moves(zero_indpos_deg,
                                                 pos,
                                                 vmax, 
                                                 accel, 
                                                 self.move_dt)
        ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)

        if noreturn == False:
            # Wait until done
            ans = raw_input('Press enter to return to zero index position:')
            # Return to the zero index position 
            print 'returning to zero index positon'
            ramps_deg = libmove_motor.get_ramp_moves(pos,
                                                     zero_indpos_deg,
                                                     vmax, 
                                                     accel, 
                                                     self.move_dt)
            ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
            end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)

    def move_by_ind(self, name_list, ind_list, noreturn=False,vmax=DFLT_MOVE_VMAX, accel=DFLT_MOVE_ACCEL): 
        """ 
        Move motor given by motor_name by the specified number of indices.  
        """
        config=self.config_dict
        n = self.num_motors()
        zero_pos = scipy.zeros((n,))
        next_pos = scipy.zeros((n,))
        if len(ind_list) == len(name_list):
            for ind, name in zip(ind_list,name_list):
                motor_num = self.get_motor_num(name)
                next_pos[motor_num] = ind 
        elif len(ind_list) == 1:
            ind = ind_list[0]
            for name in name_list:
                motor_num = self.get_motor_num(name)
                next_pos[motor_num] = ind 
        else:
            raise ValueError, 'len(pos_list) must equal len(name_list) or 1'
        
        ramp_move = libmove_motor.get_ramp_moves(zero_pos,
                                               next_pos,
                                               vmax,
                                               accel,
                                               self.move_dt)
        ramp_move = libmove_motor.convert2int(ramp_move)
        end_pos, ret_val = libmove_motor.outscan_kine(ramp_move,config,self.move_dt)

        if noreturn == False:
            # Wait until done
            ans = raw_input('Press enter to return to starting position:')
            print 'returning to starting positon'
        
            ramp_move = libmove_motor.get_ramp_moves(zero_pos,
                                                   -next_pos,
                                                   vmax,
                                                   accel,
                                                   self.move_dt)
            ramp_move = libmove_motor.convert2int(ramp_move)
            end_pos, ret_val = libmove_motor.outscan_kine(ramp_move,config,self.move_dt)

    def run(self, kine_deg=None):
        """
        Run yaw force-feedback function.
        """
        if kine_deg == None:
            if self.kine_deg == None:
                raise RuntimeError, 'no kinematics to run'
            else:
                kine_deg = self.kine_deg
            
        config = self.config_dict

        # Move to kinematics starting positon
        zero_indpos_deg = libmove_motor.get_zero_indpos_deg(self.motor_maps)
        start_pos_deg = kine_deg[0,:]
        ramps_deg = libmove_motor.get_ramp_moves(zero_indpos_deg,
                                                 start_pos_deg,
                                                 self.move_vmax, 
                                                 self.move_accel, 
                                                 self.move_dt)
        ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)

        # Sleep to wait for force transients to die down - can effect autozero
        time.sleep(self.pause_t)

        # Run force-feed back function
        kine_ind = libmove_motor.deg2ind(kine_deg, self.motor_maps)
        t, pos, vel, torq, end_pos_ind = libyawff.yawff_c_wrapper(kine_ind, config)
        
        # Convert from radians to degrees
        pos = RAD2DEG*pos
        vel = RAD2DEG*vel

        # Extract filtered and raw torque data
        torq_flt = torq[:,0]
        torq_raw = torq[:,1]
        
        # Return to zero position
        end_pos_deg = libmove_motor.ind2deg(end_pos_ind,self.motor_maps)
        ramps_deg = libmove_motor.get_ramp_moves(end_pos_deg,
                                                 zero_indpos_deg,
                                                 self.move_vmax, 
                                                 self.move_accel, 
                                                 self.move_dt)
        ramps_ind = libmove_motor.deg2ind(ramps_deg, self.motor_maps)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps_ind,config,self.move_dt)
        
        # Reset all pwm signal to there default positions. This is a kludge to help
        # w/ the fact that we seem to sometimes lose a bit of position in the pwm 
        # signals. I am not sure what is causing this - hardware or software. 
        # I will need to to some more tests after this next set of experiments.
        clkdirpwm.set_pwm_to_default(None)

        return t, pos, vel, torq_flt, torq_raw

    def num_motors(self):
        """
        Returns the number of motors
        """
        motor_num_list = libmove_motor.get_motor_num_list(self.motor_maps)
        return len(motor_num_list)

    def get_motor_num(self,name):
        """
        Returns motor number given the motor name
        """
        return self.motor_maps[name]['number']

    def set_zero_kine(self, T):
        config = self.config_dict
        dt = config['dt']
        num_motors = self.num_motors()
        t = scipy.arange(0.0, (T+dt)/dt)*dt
        kine = scipy.zeros((t.shape[0],num_motors))
        self.kine_deg = kine
        self.t = t
    
    def set_stroke_tilt_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=0.01, k_rotation=1.5, rotation_offset=0.0):
        """
        Sets current kinemtics to those with differential tilt in stroke-plane angle determined by 
        control signal u.
        """
        u_0, u_1 = u, -u
        ro_0, ro_1 = -rotation_offset, rotation_offset
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0],num_motors))
        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        kine[:,s0] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,s1] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,r0] = get_rotation_angle(t, f, amp_rotation, ro_0, k_rotation)
        kine[:,r1] = get_rotation_angle(t, f, amp_rotation, ro_1, k_rotation)
        kine[:,d0] = get_deviation_angle(t, f, u_0) 
        kine[:,d1] = get_deviation_angle(t, f, u_1) 
        self.kine_deg = kine
        self.t = t

    def set_diff_aoa_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=0.01, k_rotation=1.5, rotation_offset=0.0):
        """
        Sets current kinematics to those with a differential angle of attach on the up stroke
        and downstroke. The amount of asymmetry is determined by the control input u.
        """
        u_0, u_1 = -u, u
        ro_0, ro_1 = -rotation_offset, rotation_offset
        num_motors = self.num_motors()
        kine = scipy.zeros((t.shape[0],num_motors))
        s0 = self.get_motor_num('stroke_0')
        s1 = self.get_motor_num('stroke_1')
        r0 = self.get_motor_num('rotation_0')
        r1 = self.get_motor_num('rotation_1')
        d0 = self.get_motor_num('deviation_0')
        d1 = self.get_motor_num('deviation_1')
        kine[:,s0] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,s1] = get_stroke_angle(t, f, amp_stroke, 0.5, k_stroke)
        kine[:,r0] = get_rotation_angle(t, f, amp_rotation, u_0 + ro_0, k_rotation)
        kine[:,r1] = get_rotation_angle(t, f, amp_rotation, u_1 + ro_1, k_rotation)
        kine[:,d0] = get_deviation_angle(t, f, 0.0) 
        kine[:,d1] = get_deviation_angle(t, f, 0.0) 
        self.kine_deg = kine
        self.t = t

    def set_yaw_to_const_vel(self,vel,accel):
        """
        Sets the kinematics of the yaw motor to constant velocity.
        """
        if self.t == None or self.kine_deg == None:
            raise RuntimeError, 'cannot set yaw to constant vel - no t or kine_deg'
        n = self.get_motor_num('yaw')
        self.kine_deg[:,n] = ramp_to_const_vel(self.t,vel,accel)

    def set_yaw_to_ramp(self,x0,x1,vmax,a):
        """
        Sets kinematics of the yaw motor to a point to point ramp
        """
        num_motors = self.num_motors()
        n = self.get_motor_num('yaw')
        dt = self.config_dict['dt']
        ramp = libmove_motor.get_ramp(x0,x1,vmax,a,dt,output='ramp only')
        print ramp.shape
        if self.kine_deg == None:
            self.kine_deg = scipy.zeros((ramp.shape[0],num_motors))
            self.t = scipy.arange(0,ramp.shape[0])*dt
            self.kine_deg[:,n] = ramp
        else:
            raise RuntimeError, 'set_yaw_to_ramp w/ existing kinematics not implemented yet'

    def set_yaw_to_const(self,val):
        """
        Set yaw kinematics to constant position
        """
        num_motors = self.num_motors()
        n = self.get_motor_num('yaw')
        self.kine_deg[:,n] = val*scipy.ones((self.kine_deg.shape[0],))

    def set_cable_test_kine(self,yaw_range,yaw_step,T_meas,dt):
        """
        Set kinematics for testing for cable effects 
        """
        num_motors = self.num_motors()
        n = self.get_motor_num('yaw')
        t, yaw_kine, yaw_meas_pos = get_cable_test_kine(yaw_range,yaw_step,T_meas,dt)
        self.kine_deg = scipy.zeros((yaw_kine.shape[0],num_motors))
        self.kine_deg[:,n] = yaw_kine
        self.t = t
        return yaw_meas_pos
       
    def plot_kine(self,kine_deg=None):
        """
        Plot wing kinematics
        """
        if kine_deg == None:
            if self.kine_deg == None:
                raise RuntimeError, 'no kinematics to plot'
            else:
                kine_deg = self.kine_deg

        dt = self.config_dict['dt']
        t = scipy.arange(0.0,kine_deg.shape[0])*dt

        for motor, map in self.motor_maps.iteritems():
            ind = map['number']
            try:
                side_num = int(motor[-1])
                name = motor[:-2]
            except:
                side_num = None
                name = motor

            # Select figure    
            if side_num == 0:
                pylab.figure(1)
            elif side_num == 1:
                pylab.figure(2)
            else:
                pylab.figure(3)

            # Select subplot
            if name == 'stroke':
                pylab.subplot(3,1,1)
                pylab.title('side num: %d'%(side_num,))
                pylab.ylabel('stroke')
            elif name == 'rotation':
                pylab.subplot(3,1,2)
                pylab.ylabel('rotation')
            elif name == 'deviation':
                pylab.subplot(3,1,3)
                pylab.ylabel('deviation')
                pylab.ylabel('t')
            elif name == 'yaw':
                pylab.ylabel('yaw')
                pylab.xlabel('t')
            pylab.plot(t,kine_deg[:,ind])
        pylab.show()


def read_defaults(defaults_file=BORFRC_DIR):
    """
    Read borfrc defaults file for default motor maps, sensor calibration
    and comedi configuration files.
    """
    config = ConfigParser.ConfigParser()
    if os.path.exists(defaults_file):
        config.read(defaults_file)
    else:
        raise IOError, 'defaults file does not exist'

    if not 'yawff' in config.sections():
        raise RuntimeError, 'yawff section not found in defaults file: %s'%(defaults_file,)

    file_dict = {}
    for f in config.options('yawff'):
        val = config.get('yawff',f)
        file_dict[f] = val

    if not file_dict.has_key('motor_maps'):
        raise RuntimeError, 'defaults file does not specify motor maps file'
    if not file_dict.has_key('sensor_cal'):
        raise RuntimeError, 'defaults file does not specify sensor cal file'
    if not file_dict.has_key('comedi_conf'):
        raise RuntimeError, 'defaults file does not specify comedi conf file'

    motor_maps_file = file_dict['motor_maps']
    sensor_cal_file = file_dict['sensor_cal']
    comedi_conf_file = file_dict['comedi_conf']
    return motor_maps_file, sensor_cal_file, comedi_conf_file

def read_sensor_cal(sensor_cal_file,sensor_cal_dir=DFLT_SENSOR_CAL_DIR): 
    """
    Read sensor calibration file.
    """
    config = ConfigParser.ConfigParser()
    if os.path.exists(sensor_cal_file):
        config.read(sensor_cal_file)
    else:
        sensor_cal_file = os.path.join(sensor_cal_dir,sensor_cal_file)
        if os.path.exists(sensor_cal_file):
            config.read(sensor_cal_file)
        else:
            raise RuntimeError, 'sensor cal file not found'

    if not 'sensor_0' in config.sections():
        raise RuntimeError, 'sensor_0 in found in calibration file: %s'%(sensor_cal_file,)

    sensor_cal = {}
    for opt in config.options('sensor_0'):
        val = config.get('sensor_0', opt)
        sensor_cal[opt] = val

    sensor_cal['ain_chan'] = int(sensor_cal['ain_chan'])
    sensor_cal['calibration'] = float(sensor_cal['calibration'])
    return sensor_cal

def read_comedi_conf(comedi_conf_file, comedi_conf_dir=DFLT_COMEDI_CONF_DIR):
    """
    Read comedi configuration file.
    """
    config = ConfigParser.ConfigParser()
    if os.path.exists(comedi_conf_file):
        config.read(comedi_conf_file)
    else:
        comedi_conf_file = os.path.join(comedi_conf_dir,comedi_conf_file)
        if os.path.exists(comedi_conf_file):
            config.read(comedi_conf_file)
        else:
            raise RuntimeError, 'comedi conf file not found'

    if not 'device_0' in config.sections():
        raise RuntimeError, 'device_0 not found in comedi configuration file: %s'%(comedi_conf_file,)

    comedi_conf = {}
    for opt in config.options('device_0'):
        val = config.get('device_0', opt)
        comedi_conf[opt] = val

    comedi_conf['ain_subdev'] = int(comedi_conf['ain_subdev'])
    comedi_conf['dio_subdev'] = int(comedi_conf['dio_subdev'])
    comedi_conf['dio_disable'] = int(comedi_conf['dio_disable'])
    return comedi_conf
    
# Wing kinematics functions ------------------------------------------------------

def get_stroke_angle(t, f, amp, u, k, phase=0.5*PI):
    """
    Stroke position function w/ J. Wang's shape parameter. The control input 
    adds a asymmetric velicty to the stroke positon fuction.

    Arguments:
        t     = array of time points (s)
        f     = flapping frequency (Hz)
        amp   = stroke amplitude 
        u     = control input, scalar or array
        k     = shape parameter near 0 => sine wave, near 1=> sawtooth
    """
    f1=f/(2*u)
    f2=1/(2/f-1/f1)
    phi_A=amp/scipy.arcsin(k)*scipy.arcsin(k*scipy.sin(2*PI*f1*scipy.fmod(t, 1/f)+phase))
    phi_B=amp/scipy.arcsin(k)*scipy.arcsin(k*scipy.sin(2*PI*f2*(1/(2*f2)-1/(2*f1)+scipy.fmod(t,1/f))+phase))
    phi=phi_A*(scipy.fmod(t, 1/f)<1/(2*f1))+phi_B*(scipy.fmod(t, 1/f)>=1/(2*f1))
    return phi

def get_rotation_angle(t, f, amp, u, k, phase=0.0):
    """
    Rotation angle function w/ J. Wang's shape parameter. The control input shifts
    the mean rotation angle.

    Arguments:
        t     = array of time points 
        f     = flapping frequency
        amp   = rotation angle amplitude
        u     = control input, scalar or array
        k     = shape parameter, k near 0 implies sinewave, k large implies
                square wave.
    """
    phase = 0.0
    alpha=amp/scipy.tanh(k)*scipy.tanh(k*scipy.sin(2*PI*f*t+phase))+u
    return alpha

def get_deviation_angle(t, f, u, phase=0.0):
    """
    Deviation angle function w/ control input u.

    Arguments:
        t     = array of time points
        f     = flapping frequency
        amp   = deviation angle amplitude
        u     = control input, scalar or array
    """
    phase = 0.0
    theta = u*scipy.cos(2*PI*f*t + phase)
    return theta

def control_step(t,u0,u1,t0,t1,t2,t3):
    """
    Control function, has value u0 for t < t0, linearly transitions from u0 
    to u1 when t0 < t < t1, has value u1 when t1 < t < t2, linearly transistions
    from u1 to u0 when t2 < t < t3, and has value u0 when t > t3.
    """
    f = scipy.zeros(t.shape)
    # Masks for selecting time sections
    mask0 = t < t0
    mask1 = scipy.logical_and(t >= t0, t < t1)
    mask2 = scipy.logical_and(t >= t1, t < t2)
    mask3 = scipy.logical_and(t >= t2, t < t3)
    mask4 = t >= t3
    # Constants for linear transition regions
    a_01 = (u0 - u1)/(t0 - t1)
    a_23 = (u1 - u0)/(t2 - t3)
    b_01 = u0 - a_01*t0
    b_23 = u1 - a_23*t2
    # Assign functin values
    f[mask0] = u0*scipy.ones(t[mask0].shape)
    f[mask1] = a_01*t[mask1] + b_01
    f[mask2] = u1*scipy.ones(t[mask2].shape)
    f[mask3] = a_23*t[mask3] + b_23
    f[mask4] = u0*scipy.ones(t[mask4].shape)
    return f

def sqr_wave(t,amp,T,epsilon):
    """
    Generates a square wave with the given frequency and amplitude
    """
    f = scipy.zeros(t.shape)
    t_curr = 0.5*T
    cnt = 0
    while (t_curr - T < t[-1]):
        if cnt%2 == 0:
            val = amp
        else:
            val = -amp

        t0 = t_curr - 0.5*T 
        t1 = t_curr - 0.5*T + 0.5*epsilon
        mask0 = scipy.logical_and(t >= t0, t < t1)
        interp_func = scipy.interpolate.interp1d([t0,t1],[0,val])
        f[mask0] = interp_func(t[mask0])

        t0 = t_curr - 0.5*T + 0.5*epsilon
        t1 = t_curr - 0.5*epsilon
        mask1 = scipy.logical_and(t >= t0, t < t1)
        f[mask1] = val

        t0 = t_curr - 0.5*epsilon
        t1 = t_curr
        mask2 = scipy.logical_and(t >= t0, t < t1)
        interp_func = scipy.interpolate.interp1d([t0,t1],[val,0])
        f[mask2] = interp_func(t[mask2])

        t_curr += 0.5*T
        cnt += 1
    return f

def ramp_to_const_vel(t,vel,accel):
    """
    Generates a ramp trajectory to constant velocity.
    """
    accel = abs(accel)
    if vel < 0:
        accel = -accel
    x = scipy.zeros(t.shape)
    t_accel = vel/accel
    mask0 = t < t_accel
    mask1 = t >= t_accel
    x[mask0] = accel*t**2
    x[mask1] = vel*t + accel*(t_accel**2)
    return x

def get_cable_test_kine(yaw_range,yaw_step,T_meas,dt,max_vel=10.0,accel=10.0):
    """
    Get yaw kinematics for testing for cable effects 
    
    Inputs:
        yaw_range = range for measurement sample points
        yaw_step  = step size between measurement samples
        T_meas    = sample duration
        dt        = sample rate

    Output:
        t         = array of time points 
        yaw_kine  = array of yaw kinematics
        pos_array = array of yaw measurement positions
    """
    N_meas = int(T_meas/dt)
    pos_array = scipy.arange(-0.5*yaw_range,0.5*yaw_range+yaw_step,yaw_step)
    pos_last = 0.0
    for i, pos in enumerate(pos_array):
        # Create ramp to position
        ramp = libmove_motor.get_ramp(pos_last, pos, max_vel, accel, dt) 
        if i == 0:
            yaw_kine = ramp
        else:
            yaw_kine = scipy.concatenate((yaw_kine,ramp))
        # Dwell at positoin for measurement
        dwell = pos*scipy.ones((N_meas,))
        yaw_kine = scipy.concatenate((yaw_kine,dwell))
        pos_last =  pos
    t = scipy.arange(0,yaw_kine.shape[0])*dt
    return t, yaw_kine, pos_array
    
def resample(x,n):
    """
    Resample data in array x with step size n.
    """
    if len(x.shape) == 1:
        return x[0:-1:n]
    else:
        return x[0:-1:n,:]


# ------------------------------------------------------------------------------
# Command line utilties

class yawff_cmd_line:

    move2pos_help = """\
command: move2pos 

usage: %prog [options] move2pos motor_0, ..., motor_k, pos

       or

       %prog [options] move2pos motor_0, ..., motor_k, pos_0, ... pos_k

Move motors, specified by name, to the given positions. If one position value
is given then all motors specified are moved to that position. Otherwise the
number of positions specified must equal the number of motors and each motor
is moved to the corresponding position which is determined by the order. By 
default, after the move, the routine will prompt the user press enter and then 
return  the motors to the zero position unless the noreturn option is specified.
"""

    move_by_ind_help = """\
command: move-by-ind 

usage: %prog [options] move-by-ind motor_0, ..., motor_k, pos
 
       or

       %prog [options] move-by-ind motor_0, ..., motor_k, pos_0, ..., pos_k

Move motors, specified by name, by the given number of indices. If one value 
is given then all motors specified are moved by that number of indices. Otherwise 
the number of values specified must equal the number of motors and each motor
is moved by the corresponding value which is determined by the order. By 
default, after the move, the routine will prompt the user press enter and then 
return  the motors to the zero position unless the noreturn option is specified.
"""

    reset_pwm_help = """\
command: reset-pwm

usage: %prog [options] reset-pwm 

Resets all pwm signal to their default (start-up) positions.
"""

    motor_names_help = """\
command: motor-names

usage: %prog [options] motor-names

Displays a list of all motors names.
"""

    sensor_cal_help = """\
command: sensor-cal
    
This command has not been implemented yet.
"""

    help_help = """\
command: help

usage: %prog [options] help [COMMAND]

Prints help information. If the optional argument COMMAND is not given
then general usage information for the %prog is displayed. If a specific 
command, COMMAND, is given then help for that command will be displayed.

Examples:
 %prog help         # prints general usage information
 %prog help status  # prints help for the status command 
"""

    usage = """%prog [OPTION] command [arg0, ...] 

%prog is a command line utility providing simple positioning and other useful 
commands for working with the yawff controlled robot. This utitily is intended 
for aiding the experimenter with zeroing and calibration.

Commands:

    move2pos      - move motor to specified position
    move-by-ind   - move motor by specified number of indices 
    reset-pwm     - reset pwm output to their default positions
    motor-names   - list motor names
    sensor-cal    - calibrate torque sensor
    help          - get help  
     
* To get help for a specific command type: %prog help COMMAND
"""

    def __init__(self):
        self.cmd_table = {
            'move2pos': self.move2pos,
            'move-by-ind': self.move_by_ind,
            'reset-pwm': self.reset_pwm, 
            'motor-names': self.motor_names,
            'sensor-cal': self.sensor_cal,
            'help': self.help,
        }
        self.help_table = {
            'move2pos': yawff_cmd_line.move2pos_help,
            'move-by-ind': yawff_cmd_line.move_by_ind_help,
            'reset-pwm': yawff_cmd_line.reset_pwm_help,
            'motor-names': yawff_cmd_line.motor_names_help,
            'sensor-cal': yawff_cmd_line.sensor_cal_help,
            'help': yawff_cmd_line.help_help,
        }

        self.progname = os.path.split(sys.argv[0])[1]
        self.options_cmd, self.args, self.parser = self.parse_cmd_options()
        
        self.run_params = {
                'dt'                : 1.0/5000.0, 
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
        

    def parse_cmd_options(self):
        """
        Parse command line options 
        """

        parser = optparse.OptionParser(usage=yawff_cmd_line.usage)

        parser.add_option('-v', '--verbose',
                               action='store_true',
                               dest = 'verbose',
                               help = 'verbose mode - print additional information',
                               default = False)

        parser.add_option('-n', '--noreturn',
                               action='store_true',
                               dest = 'noreturn',
                               help = 'noreturn mode - do not return to starting positin after making move',
                               default = False)

        options, args = parser.parse_args()

        # Convert options to dictionary
        options = options.__dict__
        return options, args, parser

    def run(self):
        """
        Run command given on the command line
        """
        if len(self.args) == 0:
            print "ERROR: no command given"
            print 
            self.parser.print_help()
            sys.exit(0)
        else:
            cmd_str = self.args[0]
            try:
                cmd = self.cmd_table[cmd_str]
            except KeyError:
                print "ERROR: command, '%s', not found"%(cmd_str,)
                print 
                self.parser.print_help()
                sys.exit(1)
            # Run command
            cmd()
        return

    def help(self):
        """
        Print help messages
        """
        if len(self.args)==1:
            self.parser.print_help()
        elif len(self.args)==2:
            cmd_str = self.args[1].lower()
            try:
                help_str = self.help_table[cmd_str]
            except KeyError:
                print "ERROR: can't get help unkown command"
                sys.exit(1)
            print help_str.replace('%prog', self.progname)
        else:
            print "ERROR: too many arguments for command help"
            sys.exit(1)

    def move2pos(self):
        self.args.remove('move2pos')
        motor_names, values = self.get_motors_and_value_from_args()
        if self.options_cmd['verbose'] == True:
            if len(motor_names) == len(values):
                print 'moving motors %s to positions %s'%(motor_names,values)
            else:
                print 'moving motors %s to position %s'%(motor_names,values)
        yawff = Yawff(self.run_params)
        yawff.move_to_pos(motor_names, values, noreturn = self.options_cmd['noreturn'])

    def move_by_ind(self):
        self.args.remove('move-by-ind')
        motor_names, values = self.get_motors_and_value_from_args()
        if self.options_cmd['verbose'] == True:
            print 'moving motors %s by indices %s'%(motor_names,values)
        yawff = Yawff(self.run_params)
        yawff.move_by_ind(motor_names, values, noreturn = self.options_cmd['noreturn'])

    def reset_pwm(self):
        """
        Reset pwm outputs to default values.
        """
        if self.options_cmd['verbose'] == True:
            print 'resetting pwms to default values' 
        clkdirpwm.set_pwm_to_default(None)

    def motor_names(self):
        motor_names = self.get_motor_names()
        for name in motor_names:
            print ' ', name

    def sensor_cal(self):
        print  'sensor_cal - not implemented yet'

    def get_motor_names(self):
        yawff = Yawff(self.run_params)
        motor_names = yawff.motor_maps.keys()
        motor_names.sort()
        return motor_names

    def get_motors_and_value_from_args(self):
        motor_names = self.get_motor_names()
        motors_in_args = [x for x in self.args if x in motor_names]
        others_in_args = [x for x in self.args if x not in motor_names]
        if len(others_in_args) == 1:
            try:
                values = [float(others_in_args[0])]
            except ValueError:
                print 'ERROR: cannot cast value to float'
                sys.exit(1)
        elif len(others_in_args) == len(motors_in_args):
            values = []
            for v_str in others_in_args:
                try:
                    v = float(v_str)
                except ValueError:
                    print 'ERROR: cannot cast value to float'
                    sys.exit(1)
                values.append(v)
        else:
            print 'ERROR: incorrect number of angle values -  must equal 1 or number of motor names given' 
            sys.exit(1)
        return motors_in_args, values

def cmd_line_main():
    """
    Command line interface entry point.
    """
    cmd_line = yawff_cmd_line()
    cmd_line.run()
    pass
