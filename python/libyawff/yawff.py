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
import ctypes 
import scipy
from libmove_motor import convert2int

lib = ctypes.cdll.LoadLibrary("libyawff.so.1")

def get_c_array_struct(x):
    """
    Get the C array structure associated with the scipy or numpy array.
    """
    x_struct = array_t()
    x_struct.data = x.ctypes.data_as(ctypes.c_void_p)
    x_struct.nrow = x.ctypes.shape[0]
    x_struct.ncol = x.ctypes.shape[1]
    x_struct.s0 = x.ctypes.strides[0]
    x_struct.s1 = x.ctypes.strides[1]
    if x.dtype == scipy.dtype('int'):
        x_struct.type = INT_ARRAY
    elif x.dtype == scipy.dtype('float32'):
        x_struct.type = FLT_ARRAY
    elif x.dtype == scipy.dtype('float64'):
        x_struct.type = DBL_ARRAY
    else:
        raise ValueError, "array must be of type INT_ARRAY, FLT_ARRAY or DBL_ARRAY" 
    return x_struct

def create_config_struct(config):
    """
    Create c configuration structure
    """
    config_struct = config_t()
    config_struct.dev_name = config['dev_name']
    config_struct.ain_subdev = config['ain_subdev']
    config_struct.dio_subdev = config['dio_subdev']
    config_struct.num_motor = config['num_motor']
    config_struct.yaw_motor = config['yaw_motor']
    config_struct.dio_clk = config['dio_clk']
    config_struct.dio_dir = config['dio_dir']
    config_struct.kine_map = config['kine_map']
    config_struct.dio_disable = config['dio_disable']
    config_struct.yaw_ain = config['yaw_ain']
    config_struct.yaw_ain_zero_dt = config['yaw_ain_zero_dt']
    config_struct.yaw_ain_zero_num = config['yaw_ain_zero_num'] 
    config_struct.yaw_volt2torq = config['yaw_volt2torq']
    config_struct.yaw_inertia = config['yaw_inertia']
    config_struct.yaw_ind2deg = config['yaw_ind2deg']
    config_struct.yaw_torq_lim = config['yaw_torq_lim']
    config_struct.yaw_torq_deadband = config['yaw_torq_deadband']
    config_struct.yaw_filt_lpcut = config['yaw_filt_lpcut']
    config_struct.yaw_filt_hpcut = config['yaw_filt_hpcut']
    config_struct.yaw_damping = config['yaw_damping']
    config_struct.dt = int(S2NS*config['dt'])
    config_struct.integ_type = int(config['integ_type'])
    config_struct.startup_t = float(config['startup_t'])
    config_struct.ff_flag = int(config['ff_flag'])

    # Handle new configuration stuff when feedback controller is present.
    if config.has_key('ctlr_flag'):
        
        config_struct.ctlr_flag = int(config['ctlr_flag'])

        if config_struct.ctlr_flag != CTLR_OFF: 

            # Setup motor id map - this is more complicated than needed
            extended_map = list(config['kine_map']) + [config['yaw_motor']]
            extended_map.sort()
            id_tuple =  tuple([motor_id_dict[config['motor_name_map'][n]] for n in extended_map])
            config_struct.motor_id_map = id_tuple

            config_struct.ctlr_param.type = int(config['ctlr_type'])
            config_struct.ctlr_param.pgain = float(config['ctlr_pgain'])
            config_struct.ctlr_param.dgain = float(config['ctlr_dgain'])

            config_struct.kine_param.type = int(kine_type_id_dict[config['kine_type']])
            config_struct.kine_param.period = float(config['kine_period'])
            config_struct.kine_param.stroke_amp = float(config['stroke_amp'])
            config_struct.kine_param.rotation_amp = float(config['rotation_amp'])
            config_struct.kine_param.stroke_k = float(config['stroke_k'])
            config_struct.kine_param.rotation_k = float(config['rotation_k'])

            # Get motor calibration data
            for i,cal in enumerate(config['motor_cal']):
                if cal['type'] == 'table':
                    config_struct.motor_cal[i].type = MOTOR_CALTYPE_TBL
                    deg_data = scipy.reshape(cal['deg_data'], (cal['deg_data'].shape[0],1))
                    ind_data = scipy.reshape(cal['ind_data'], (cal['ind_data'].shape[0],1))
                    config_struct.motor_cal[i].deg_data = get_c_array_struct(deg_data)
                    config_struct.motor_cal[i].ind_data = get_c_array_struct(ind_data)
                elif cal['type'] == 'mult':
                    config_struct.motor_cal[i].type = MOTOR_CALTYPE_MUL
                    config_struct.motor_cal[i].deg_per_ind = float(cal['deg_per_ind'])

    else:
        config_struct.ctlr_flag = CTLR_OFF 

    return config_struct

# Constants 
S2NS = 1.0e9
MAX_MOTOR = lib.define_max_motor()
MAX_DT = lib.define_max_dt()
MIN_DT = lib.define_min_dt()
CLOCK_HI_NS = lib.define_clock_hi_ns()
INTEG_EULER = lib.define_integ_euler()
INTEG_RKUTTA = lib.define_integ_rkutta()
INTEG_UNKNOWN = lib.define_integ_unknown()
EMPTY_ARRAY = lib.define_empty_array()
INT_ARRAY = lib.define_int_array()
FLT_ARRAY = lib.define_flt_array()
DBL_ARRAY = lib.define_dbl_array()
UNKOWN_ARRAY = lib.define_unknown_array()
SUCCESS = lib.define_success()
FAIL = lib.define_fail()
FF_ON = lib.define_ff_on()
FF_OFF = lib.define_ff_off()
CTLR_ON = lib.define_ctlr_on()
CTLR_OFF = lib.define_ctlr_off()
CTLR_TYPE_VEL = lib.define_ctlr_type_vel()
CTLR_TYPE_POS = lib.define_ctlr_type_pos()
MOTOR_CALTYPE_TBL = lib.define_motor_caltype_tbl()
MOTOR_CALTYPE_MUL = lib.define_motor_caltype_mul()

# Motor ids
STROKE_0_ID = lib.stroke_0_id()
STROKE_1_ID = lib.stroke_1_id()
ROTATION_0_ID = lib.rotation_0_id()
ROTATION_1_ID = lib.rotation_1_id()
DEVIATION_0_ID = lib.deviation_0_id()
DEVIATION_1_ID = lib.deviation_1_id()
YAW_ID = lib.yaw_id()

# Kinematics type ids
DIFF_AOA_ID = lib.diff_aoa_id()
DIFF_DEV_ID = lib.diff_dev_id()

# Dictionary relating motor names to motor identifiers
motor_id_dict = {
    'stroke_0'    : STROKE_0_ID,
    'stroke_1'    : STROKE_1_ID,
    'rotation_0'  : ROTATION_0_ID,
    'rotation_1'  : ROTATION_1_ID,
    'deviation_0' : DEVIATION_0_ID,
    'deviation_1' : DEVIATION_1_ID,
    'yaw'         : YAW_ID,
}

kine_type_id_dict = {
    'diff_aoa' : DIFF_AOA_ID,
    'diff_dev' : DIFF_DEV_ID,
}

# Structures
class array_t(ctypes.Structure):
    _fields_ = [
        ('data', ctypes.c_void_p), 
        ('nrow', ctypes.c_int), 
        ('ncol', ctypes.c_int),
        ('s0', ctypes.c_int),
        ('s1', ctypes.c_int),
        ('type', ctypes.c_int), 
        ]

class motor_cal_t(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int),
        ('deg_per_ind', ctypes.c_float),
        ('deg_data', array_t),
        ('ind_data', array_t),
    ]

class ctlr_param_t(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int),
        ('pgain', ctypes.c_float),
        ('dgain', ctypes.c_float),
    ]

class kine_param_t(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int),
        ('period', ctypes.c_float),
        ('stroke_amp', ctypes.c_float),
        ('rotation_amp', ctypes.c_float),
        ('stroke_k', ctypes.c_float),
        ('rotation_k', ctypes.c_float),
    ]


class config_t(ctypes.Structure):
    _fields_ = [
        ('dev_name', ctypes.c_char_p),
        ('ain_subdev', ctypes.c_uint),
        ('dio_subdev', ctypes.c_int),
        ('num_motor', ctypes.c_int),
        ('yaw_motor', ctypes.c_int),
        ('dio_clk', MAX_MOTOR*ctypes.c_int),
        ('dio_dir', MAX_MOTOR*ctypes.c_int),
        ('kine_map', MAX_MOTOR*ctypes.c_int),
        ('motor_id_map', MAX_MOTOR*ctypes.c_int),
        ('dio_disable', ctypes.c_int),
        ('yaw_ain', ctypes.c_uint),
        ('yaw_ain_zero_dt', ctypes.c_float),
        ('yaw_ain_zero_num', ctypes.c_uint),
        ('yaw_volt2torq',ctypes.c_float),
        ('yaw_inertia', ctypes.c_float),
        ('yaw_ind2deg', ctypes.c_float),
        ('yaw_torq_lim', ctypes.c_float),
        ('yaw_torq_deadband', ctypes.c_float),
        ('yaw_filt_lpcut', ctypes.c_float),
        ('yaw_filt_hpcut', ctypes.c_float),
        ('yaw_damping', ctypes.c_float),
        ('dt', ctypes.c_int),
        ('integ_type', ctypes.c_int),
        ('startup_t', ctypes.c_float),
        ('ff_flag', ctypes.c_int),
        ('ctlr_flag', ctypes.c_int),
        ('ctlr_param', ctlr_param_t),
        ('kine_param', kine_param_t),
        ('motor_cal', MAX_MOTOR*motor_cal_t),
        ]


class data_t(ctypes.Structure):
    _fields_ = [
        ('t', array_t),
        ('pos', array_t),
        ('vel', array_t),
        ('torq', array_t),
        ]

# Functions 
lib.yawff.restype = ctypes.c_int
lib.yawff.argstype = [
    array_t,
    config_t,
    data_t,
    ctypes.c_void_p,
]

lib.yawff_w_ctlr.restype = ctypes.c_int
lib.yawff_w_ctlr.argstype = [
    array_t, 
    config_t, 
    array_t,
    data_t,
    ctypes.c_void_p,
]

lib.get_start_pos.restype = ctypes.c_int
lib.get_start_pos.argstype = [
    ctypes.c_float,
    array_t,
    config_t,
]


def yawff_ctlr_c_wrapper(setpt, config):
    """
    Python wrapper for yawff_w_ctlr function. Perform yaw force-feedback
    task with yaw controller.  Spawns real-time thread to handle controller,
    data acquisition, and yaw dynamics. During outscan displays information
    regarding ongoing real-time task.

    Inputs:
        setpt   = Nx1 array of setpt values at time steps 0, dt, ... (N-1)*dt
        config  = system configuration dictionary 
    """
    config_struct = create_config_struct(config)

    # Create c structure for setpt data
    setpt_float32 = setpt.astype(scipy.dtype('float32'))
    setpt_struct = get_c_array_struct(setpt_float32)

    # Create time, position, velocity, torque, and kinematics arrarys for
    # return data
    n = setpt.shape[0]
    t = scipy.zeros((n,1), dtype = scipy.dtype('float64'))
    pos = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    vel = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    torq = scipy.zeros((n,2), dtype = scipy.dtype('float32'))
    kine = scipy.zeros((n, config['num_motor']), scipy.dtype('float32'))
    u = scipy.zeros((n,1), dtype =scipy.dtype('float32'))

    # Create c data structure
    data_struct = data_t()
    data_struct.t = get_c_array_struct(t)
    data_struct.pos = get_c_array_struct(pos)
    data_struct.vel = get_c_array_struct(vel)
    data_struct.torq = get_c_array_struct(torq)
    kine_struct = get_c_array_struct(kine)
    u_struct = get_c_array_struct(u)

    # Create array for ending positions
    end_pos = (ctypes.c_int*config['num_motor'])()

    # Call C library yawff function
    ret_val = lib.yawff_w_ctlr(setpt_struct, config_struct, kine_struct, u_struct, data_struct, end_pos)
    if ret_val == FAIL:
        raise RuntimeError, "lib.yawff_w_ctlr call failed"

    end_pos = scipy.array(end_pos)

    return t, pos, vel, torq, kine, u, end_pos


def yawff_c_wrapper(kine, config):
    """
    Python wrapper for yawff function. Performs yaw force-feedback
    task. Spawns real-time thread to handle kinematics outscan, data
    acquisition, and yaw dynamics. During outscaning displays
    information regarding ongoing real-time task.
    
    Inputs:
      kine    = Nx7 array of wing kinematics in indices, one column is for 
                yaw.
      config  = system configuration dictionary 
    """

    # Convert kinematics to integers
    kine = convert2int(kine)

    config_struct = create_config_struct(config)

    # Create c kinematics array structure
    kine_int = kine.astype(scipy.dtype('int'))
    kine_struct =  get_c_array_struct(kine_int)
        
    # Time, position, velocity, and torque arrays for return data
    n = kine.shape[0]
    t = scipy.zeros((n,1), dtype = scipy.dtype('float64'))
    pos = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    vel = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    torq = scipy.zeros((n,2), dtype = scipy.dtype('float32'))

    # Create c data structure
    data_struct = data_t()
    data_struct.t = get_c_array_struct(t)
    data_struct.pos = get_c_array_struct(pos)
    data_struct.vel = get_c_array_struct(vel)
    data_struct.torq = get_c_array_struct(torq)
    
    # Create array for ending positions
    end_pos = (ctypes.c_int*config['num_motor'])()

    # Call C library yawff function
    ret_val = lib.yawff(kine_struct, config_struct, data_struct, end_pos)
    if ret_val == FAIL:
        raise RuntimeError, "lib.yawff call failed"

    end_pos = scipy.array(end_pos)

    return t, pos, vel, torq, end_pos

def get_start_pos(setpt,config):
    """
    Python wrapper for get_start_pos function. Returns starting position of
    kinematics.
    """

    config_struct = create_config_struct(config)
    kine = scipy.zeros((1,config['num_motor']), dtype=scipy.dtype('float32'))
    kine_c_array = get_c_array_struct(kine)
    setpt_c_float = ctypes.c_float(setpt)

    ret_val = lib.get_start_pos(setpt_c_float,kine_c_array,config_struct)
    if ret_val == FAIL:
        raise RuntimeError, "lib.get_start_pos call failed"

    return kine


    
