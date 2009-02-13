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

BORFRC_DIR = os.path.join(os.environ('HOME'),'.borfrc')
DLFT_CONFIGS_FILE = os.path.join(BORFRC_DIR, 'defaults')
DFLT_SENSOR_CAL_DIR = os.path.join(BORFRC_DIR,'sensor_cal')
DFLT_COMEDI_CONF_DIR = os.path.join(BORFRC_DIR, 'comedi_conf')

class Yawff:

    def __init__(self, 
                 run_params, 
                 motor_maps_file=None, 
                 sensor_cal_file=None, 
                 comedi_conf_file=None,
                 defaults_file=DFLT_CONFIGS_FILE):
        """
        Initialize Yawff class
        """
        if motor_maps_file==None or sensor_cal_file==None or comedi_conf_file=None:
            dflts = read_defaults(defaults_file)
            #dflt_motor_maps_file, dflt_sensor_cal_file, dflt_comedi_conf_file = dflts

       # # Read motor maps file
       # if motor_maps_file = None:
       #     self.motor_maps_file = dflt_motor_map_file
       # else:
       #     self.motor_maps_file = motor_maps_file
       # self.motor_maps = libmove_motor.read_motor_maps(self.motor_maps_file)

       # # Read sensor calibration file

       # # Read comedi configuration file



def read_defaults(defaults_file=BORFRC_FILE):
    """
    Read borfrc defaults file for default motor maps, sensor calibration
    and comedi configuration files.
    """
    config = ConfigParser.ConfigParser()
    if os.exist(defaults_file):
        config.read(defaults_file)
    else:
        raise IOError, 'defaults file does not exist'

    if not 'yawff' in config.sections():
        raise RuntimeError, 'yawff section not found in defaults file'

    for opt in config.options('yawff'):
        val = config.get('yawff',opt)
        print opt, val



    


def read_sensor_cal(sensor_cal_file,sensor_cal_dir=DFLT_SENSOR_CAL_DIR): 
    pass

def read_comedi_config(comedi_conf_file, DFLT_COMEDI_CONF_DIR):
    pass




