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

RAD2DEG = scipy.pi/180.0
BORFRC_DIR = os.path.join(os.environ['HOME'],'.borfrc')
DFLT_CONFIG_FILE = os.path.join(BORFRC_DIR, 'defaults')
DFLT_SENSOR_CAL_DIR = os.path.join(BORFRC_DIR,'sensor_cal')
DFLT_COMEDI_CONF_DIR = os.path.join(BORFRC_DIR, 'comedi_conf')
DFLT_MOVE_VMAX = 500
DFLT_MOVE_ACCEL = 500
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
        motor_num_list = libmove_motor.get_motor_nums(self.motor_maps)
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

    def run(self, kine):
        """
        Run yaw force-feedback function.
        """
        import pylab
        config = self.create_config_dict()

        # Convert kinematics from degrees to indices
        kine_ind = scipy.zeros(kine.shape) 
        for motor, map in self.motor_maps.iteritems():
            n = map['number']
            kine_ind[:,n] = libmove_motor.convert_deg2ind(kine[:,n],map)
            
        # Move to kinematics starting positon
        zero_pos = scipy.zeros(kine_ind[0,:].shape)
        ramps = libmove_motor.get_ramp_moves(zero_pos,kine_ind[0,:],self.move_vmax, self.move_accel, self.move_dt)
        #for i in range(0,ramps.shape[1]):
        #    pylab.plot(ramps[:,i])
        #    pylab.title('motor %d'%(i,))
        #    pylab.show()
        end_pos, ret_val = libmove_motor.outscan_kine(ramps,config,self.move_dt)

        # Run force-feed back function
        t, pos, vel, torq, end_pos = libyawff.yawff_c_wrapper(kine_ind, config)
        
        # Convert from radians to degrees
        pos = RAD2DEG*pos
        vel = RAD2DEG*vel

        # Extract filtered and raw torque data
        torq_flt = torq[:,0]
        torq_raw = torq[:,1]
        
        # Return to zero position
        ramps = libmove_motor.get_ramp_moves(end_pos,zero_pos,self.move_vmax, self.move_accel, self.move_dt)
        end_pos, ret_val = libmove_motor.outscan_kine(ramps,config,self.move_dt)

        return t, pos, vel, torq_flt, torq_raw

    def num_motors(self):
        """
        Returns the number of motors
        """
        motor_num_list = libmove_motor.get_motor_nums(self.motor_maps)
        return len(motor_num_list)


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
    




