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
    else:
        raise ValueError, "array must be of type INT_ARRAY or FLT_ARRAY" 
    return x_struct

lib = ctypes.cdll.LoadLibrary("libyawff.so.1")

# Constants 
S2NS = 1.0e9
MAX_MOTOR = lib.define_max_motor()
MAX_DT = lib.define_max_dt()
MIN_DT = lib.define_min_dt()
CLOCK_HI_NS = lib.define_clock_hi_ns()
INTEG_EULER = lib.define_integ_euler()
INTEG_RKUTTA = lib.define_integ_euler()
INTEG_UNKNOWN = lib.define_integ_unknown()
EMPTY_ARRAY = lib.define_empty_array()
INT_ARRAY = lib.define_int_array()
FLT_ARRAY = lib.define_flt_array()
UNKOWN_ARRAY = lib.define_unknown_array()
SUCCESS = lib.define_success()
FAIL = lib.define_fail()


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
        ('dio_disable', ctypes.c_int),
        ('yaw_ain', ctypes.c_uint),
        ('yaw_ain_zero_dt', ctypes.c_float),
        ('yaw_ain_zero_num', ctypes.c_uint),
        ('yaw_volt2torq',ctypes.c_float),
        ('yaw_inertia', ctypes.c_float),
        ('yaw_ind2deg', ctypes.c_float),
        ('yaw_torq_lim', ctypes.c_float),
        ('yaw_torq_deadband', ctypes.c_float),
        ('yaw_filt_cut', ctypes.c_float),
        ('yaw_damping', ctypes.c_float),
        ('dt', ctypes.c_int),
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


def yawff(kine, config):
    """
    Python wrapper for yawff function. Performs yaw force-feedback
    task. Spawns real-time thread to handle kinematics outscan, data
    acquisition, and yaw dynamics. During outscaning displays
    information regarding ongoing real-time task.
    
    Inputs:
      kine    = Nx6 array of wing kinematics in indices
      config  = system configuration dictionary 
    """

    # Create c configuration structure
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
    config_struct.yaw_filt_cut = config['yaw_filt_cut']
    config_struct.yaw_damping = config['yaw_damping']
    config_struct.dt = int(S2NS*config['dt'])

    print config_struct.dio_subdev

    # Create c kinematics array structure
    kine_int = kine.astype(scipy.dtype('int'))
    kine_struct =  get_c_array_struct(kine_int)
        
    # Time, position, velocity, and torque arrays for return data
    n = kine.shape[0]
    t = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    pos = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    vel = scipy.zeros((n,1), dtype = scipy.dtype('float32'))
    torq = scipy.zeros((n,1), dtype = scipy.dtype('float32'))

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
    
