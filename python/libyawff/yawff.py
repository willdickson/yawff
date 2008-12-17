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

lib = ctypes.cdll.LoadLibrary("libyawff.so.1")

# Constants 
MAX_MOTOR = lib.get_max_motor()
MAX_DT = lib.get_max_dt()
MIN_DT = lib.get_min_dt()
CLOCK_HI_NS = lib.get_clock_hi_ns()

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
        ('dio_sudev', ctypes.c_int),
        ('num_motor', ctypes.c_int),
        ('yaw_motor', ctypes.c_int),
        ('dio_clk', MAX_MOTOR*ctypes.c_int),
        ('dio_dir', MAX_MOTOR*ctypes.c_int),
        ('kine_map', MAX_MOTOR*ctypes.c_int),
        ('dio_disable', ctypes.c_int),
        ('yaw_ain', ctypes.c_uint),
        ('yaw_volt2torq',ctypes.c_float),
        ('yaw_inertia', ctypes.c_float),
        ('yaw_ind2deg', ctypes.c_float),
        ('yaw_torq_lim', ctypes.c_float),
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

# // Structure for return data
# typedef struct {
#   array_t t;
#   array_t pos;
#   array_t vel;
#   array_t torq;
# } data_t;
