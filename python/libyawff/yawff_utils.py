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
import os
import os.path
import ConfigParser
import libmove_motor
import libyawff
import scipy
import pylab

PI = scipy.pi
DEG2RAD = scipy.pi/180.0
RAD2DEG = 180.0/scipy.pi
BORFRC_DIR = os.path.join(os.environ['HOME'],'.borfrc')
DFLT_CONFIG_FILE = os.path.join(BORFRC_DIR, 'defaults')
DFLT_SENSOR_CAL_DIR = os.path.join(BORFRC_DIR,'sensor_cal')
DFLT_COMEDI_CONF_DIR = os.path.join(BORFRC_DIR, 'comedi_conf')
DFLT_MOVE_VMAX = 10
DFLT_MOVE_ACCEL = 100
DFLT_MOVE_DT = 1.0/3000.0

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

    
    def set_stroke_tilt_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=0.01, k_rotation=1.5, rotation_offset=0.0):
        """
        Sets current kinemtics to those with differential tilt in stroke-plane angle determined by 
        control signal u.
        """
        u_0, u_1 = u, -u
        ro_0, ro_1 = rotation_offset, -rotation_offset
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

    def set_diff_aoa_kine(self, t, f, u, amp_stroke, amp_rotation, k_stroke=0.01, k_rotation=1.5, rotation_offset=0.0):
        """
        Sets current kinematics to those with a differential angle of attach on the up stroke
        and downstroke. The amount of asymmetry is determined by the control input u.
        """
        u_0, u_1 = u, -u
        ro_0, ro_1 = rotation_offset, -rotation_offset
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
    a = (u0 - u1)/(t0 - t1)
    b_01 = u0 - a*t0
    b_23 = u1 + a*t2
    # Assign functin values
    f[mask0] = u0*scipy.ones(t[mask0].shape)
    f[mask1] = a*t[mask1] + b_01
    f[mask2] = u1*scipy.ones(t[mask2].shape)
    f[mask3] = -a*t[mask3] + b_23
    f[mask4] = u0*scipy.ones(t[mask4].shape)
    return f

    
def resample(x,n):
    """
    Resample data in array x with step size n.
    """
    if len(x.shape) == 1:
        return x[0:-1:n]
    else:
        return x[0:-1:n,:]
