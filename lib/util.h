/*---------------------------------------------------------------------
  yawff
  Copyright (C) William Dickson, 2008.
  
  wbd@caltech.edu
  www.willdickson.com

  Released under the LGPL Licence, Version 3
  
  This file is part of yawff.

  yawff is free software: you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.
    
  yawff is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General
  Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with yawff.  If not, see
  <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------
  util.h

  Purpose: 

  Author: Will Dickson 
----------------------------------------------------------------------- */
#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "yawff.h"

#define PRINT_ERR_MSG(ERR_MSG) (print_err_msg(__FILE__,__LINE__,__FUNCTION__,ERR_MSG))

// Integrator - integrate yaw dynamic state one time step
extern int integrator(state_t state_curr, 
		      state_t *state_next, 
		      float force, 
		      float mass, 
		      float damping, 
		      float dt, 
		      int method);

// First order lowpass filter
extern float lowpass_filt1(float x,
			   float y, 
			   float f_cut, 
			   float dt);

// Initialize array structure memory
extern int init_array(array_t *array, 
		      int nrow, 
		      int ncol, 
		      int type);

// Free array structure memory
extern void free_array(array_t *array);

// Get array value at given row and column indices
extern int get_array_val(array_t array, 
			 int row, 
			 int col, 
			 void *val);

// Set array value at given row and column indices
extern int set_array_val(array_t array, 
			 int row, 
			 int col, 
			 void *val);

// Print system configuration structure
extern void print_config(config_t config);

// Print formated error message
extern void print_err_msg(const char *file, 
			  int line, 
			  const char *func, 
			  char *err_msg);

// Print and then fflush buffer
extern void fflush_printf(const char *format, ...);

// Function wich return constants for Python ctypes wrapper
extern int get_max_motor(void);
extern int get_max_dt(void);
extern int get_min_dt(void);
extern int get_clock_hi_ns(void);

#endif // INC_UTIL_H_
