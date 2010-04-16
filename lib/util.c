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
  util.c

  Puropse: contains vairous utility functions used by the yawff
  library.

  Functions:
 
   integrator         = 2nd order integrator
   dynamics_func      = function for yaw dynamics
   init_array         = initializes array object
   free_array         = frees array object
   set_array_val      = sets array element value
   get_array_val      = gets array  element value
   print_array        = prints array values
   apply_motor_cal    = applies motor calibration
   interp             = array interpolation
   lowpass_filt1      = firat order lowpass filter
   print_config       = prints configuration structure
   print_err_msg      = prints error messages
   fflush_printf      = printf followed by fflush(stdout)
   get_max_motor      = returns maximum allowed number of motors
   get_max_dt         = returns maximum allowed real-time period
   get_min_dt         = returns minimum allowed real-time period
   get_clock_hi_ns    = returns the clock high time in ns

  Author: Will Dickson
----------------------------------------------------------------------- */
#include "yawff.h"
#include "util.h"

// Function prototypes
state_t dynamics_func(
    state_t state_in, 
    float force, 
    float mass, 
    float damping
    );

// ----------------------------------------------------------------
// Function: integrator
//
// Purpose: integrator for turning yaw torque into position. 
// Currently can select bewteen 1st order Euler method and 4-5th 
// order Runge-Kutta. 
//
// Arguments:
//   state_curr  = dynamic state (position + velocity) at current 
//                 time step
//   state_next  = dynamic state a the next time step (output) 
//   force       = external force or torque 
//   mass        = mass or moment of inertia
//   damping     = damping constant
//   dt          = time step
//   method      = integration method (INTEG_EULER or INTEG_RKUTTA)
//
// Return: SUCCESS or FAIL
// 
// ----------------------------------------------------------------
int integrator(
    state_t  state_curr,
    state_t *state_next,
    float force, 
    float mass, 
    float damping, 
    float dt, 
    int method
    )
{
  state_t x;
  state_t k0,k1,k2,k3;
  int ret_flag = SUCCESS;

  switch (method) {

  case INTEG_RKUTTA: // Integrate using Runge-Kutta method
    
    x.pos = state_curr.pos;
    x.vel = state_curr.vel;
    k0 = dynamics_func(x,force,mass,damping);
  
    x.pos = state_curr.pos + k0.pos*dt/2.0;
    x.vel = state_curr.vel + k0.vel*dt/2.0;
    k1 = dynamics_func(x,force,mass,damping);
  
    x.pos = state_curr.pos + k1.pos*dt/2.0;
    x.vel = state_curr.vel + k1.vel*dt/2.0;
    k2 = dynamics_func(x,force,mass,damping);
  
    x.pos = state_curr.pos + k2.pos*dt;
    x.vel = state_curr.vel + k2.vel*dt;
    k3 = dynamics_func(x,force,mass,damping);
  
    state_next -> pos = state_curr.pos + (dt/6.0)*(k0.pos + 2.0*k1.pos + 2.0*k2.pos + k3.pos);
    state_next -> vel = state_curr.vel + (dt/6.0)*(k0.vel + 2.0*k1.vel + 2.0*k2.vel + k3.vel);
    break;
    
  case INTEG_EULER: // Integrate using Euler method
    
    k0 = dynamics_func(state_curr,force,mass,damping);
    state_next -> pos = state_curr.pos + dt*k0.pos;
    state_next -> vel = state_curr.vel + dt*k0.vel;
    break;
    
  default: // Error unkown integration method
    
    PRINT_ERR_MSG("unknown integration method");
    ret_flag = FAIL;
    break;

  }
  return ret_flag;

} 

// ----------------------------------------------------------------
// Function: dynamics_func
//
// Purpose: implements function governing the dynamics of the 
// system, i.e., the function f such that  dx/dt = f(x,t) where
// x is the system state.  
// 
// ----------------------------------------------------------------
state_t dynamics_func(state_t state_in, float force, float mass, float damping)
{
  state_t state_out;
  state_out.pos = state_in.vel;
  // Square law drag
  //state_out.vel = force/mass - (damping/mass)*state_in.vel*fabsf(state_in.vel);
  // Linear drag
  state_out.vel = force/mass - (damping/mass)*state_in.vel;
  return state_out;
}


// ----------------------------------------------------------------
// Function: lowpass_filt1
//
// Purpose: Discrete implementation of a simple first order lowpass 
// filter.
//
// Arguments:
// 
//   x       = current input value
//   y_old   = last output value
//   f_cut   = cutoff frequency in (Hz)
//   dt      = Update time step
//
// Return: new filter output y value
// 
// ----------------------------------------------------------------
float lowpass_filt1(float x,float y_old, float f_cut, float dt)
{
  float tc = 1.0/(2.0*M_PI*f_cut);
  float y_new;
  float alpha;
  alpha = dt/(tc + dt);
  y_new = alpha*x  + (1.0 - alpha)*y_old;
  return y_new;
}

// -----------------------------------------------------------------
// Function: highpass_filt1
//
// Purpose: Discrete implementation of a simple first order highpass
// filter.
//
// Arguments:
//
// Return filter ouput value.
//
// -----------------------------------------------------------------
float highpass_filt1(float dx,  float y_old, float f_cut, float dt)
{
  float tc = 1.0/(2.0*M_PI*f_cut);
  float y_new;
  float alpha;
  alpha = tc/(tc + dt);
  y_new = alpha*y_old + alpha*dx;
  return y_new;
}

// ----------------------------------------------------------------
// Function: get_array_val
//
// Puropse: gets value of array at given row and column
//
// -----------------------------------------------------------------
int get_array_val(array_t array, int row, int col, void *val)
{
  int s0, s1;
  int *iptr;
  float *fptr;
  double *dptr;
  int rtn_val = SUCCESS;

  // Check row and col ranges
  if ((row < 0) || (row >= array.nrow)) {
    PRINT_ERR_MSG("row out of range");
    return FAIL;
  }
  if ((col < 0) || (col >= array.ncol)) {
    PRINT_ERR_MSG("col out of range");
    return FAIL;
  }

  // Get array value
  s0 = array.s0;
  s1 = array.s1;

  switch(array.type) {
    
  case INT_ARRAY:
    iptr = val;
    *iptr = *((int*)(array.data + row*s0 + col*s1));
    break;

  case FLT_ARRAY:
    fptr = val;
    *fptr = *((float*)(array.data + row*s0 + col*s1));
    break;

  case DBL_ARRAY:
    dptr = val;
    *dptr = *((double*)(array.data + row*s0 + col*s1));
    break;

  default:
    PRINT_ERR_MSG("unknown array type");
    rtn_val = FAIL;
    break;

  }
  return rtn_val;
}

// -------------------------------------------------------------------
// Function: set_array_val
//
// Purpose: sets the value of an array at the given row
// and column.
//
// --------------------------------------------------------------------
int set_array_val(array_t array, int row, int col, void *val)
{
  int s0, s1;
  int *iptr;
  float *fptr;
  double *dptr;
  int rtn_val = SUCCESS;

  // Check row and col ranges
  if ((row < 0) || (row >= array.nrow)) {
    PRINT_ERR_MSG("row out of range");
    return FAIL;
  }
  if ((col < 0) || (col >= array.ncol)) {
    PRINT_ERR_MSG("col out of range");
    return FAIL;
  }
  
  // Set array value
  s0 = array.s0;
  s1 = array.s1;

  switch(array.type) {

  case INT_ARRAY:
    iptr = val;
    *((int*)((array.data) + row*s0 + col*s1)) = *iptr;
    break;

  case FLT_ARRAY:
    fptr = val;
    *((float*)((array.data) + row*s0 + col*s1)) = *fptr;    
    break;

  case DBL_ARRAY:
    dptr = val;
    *((double*)((array.data) + row*s0 + col*s1)) = *dptr;    
    break;

  default:
    PRINT_ERR_MSG("unknown array type");
    rtn_val = FAIL;
    break;
    
  }
  return rtn_val;
}
// -----------------------------------------------------
// Function: print_array
//
// Purpose: prints array values
// -----------------------------------------------------
void print_array(array_t array)
{
  int i,j;
  int ival;
  float fval;
  double dval;

  printf("size: (%d,%d)\n", array.nrow,array.ncol);
  printf("type: ");
  switch(array.type) {
    case INT_ARRAY:
      printf("int\n");
      break;
    case FLT_ARRAY:
      printf("float\n");
      break;
    case DBL_ARRAY:
      printf("double\n");
      break;
    default:
      break;
  }

  for (i=0; i<array.nrow; i++) {
    for (j=0; j<array.ncol; j++) {

      printf("i: %d, ", i);

      switch(array.type) {

        case INT_ARRAY:
          get_array_val(array,i,j,&ival);
          printf("%d, ",ival);
          break;

        case FLT_ARRAY:
          get_array_val(array,i,j,&fval);
          printf("%f, ",fval);
          break;

        case DBL_ARRAY:
          get_array_val(array,i,j,&dval);
          printf("%f, ",dval);
          break;

        default:
          PRINT_ERR_MSG("unknown array type");
          break;

      }
    }
    printf("\n");
  }
  return;
}

// ----------------------------------------------------
// Function: apply_motor_cal
//
// Purpose: apply motor calibration to convert degrees
// to motor indices
// ----------------------------------------------------
int apply_motor_cal(
    motor_cal_t motor_cal, 
    double val_deg, 
    int *val_ind
    )
{
  int rtn_val = SUCCESS;
  int rtn_check;
  double val_ind_dbl = 0.0;

  switch(motor_cal.type) {

    case MOTOR_CALTYPE_TBL:
      rtn_check = interp(motor_cal.deg_data, motor_cal.ind_data, val_deg, &val_ind_dbl); 
      if (rtn_check == FAIL) {
        PRINT_ERR_MSG("interp failed");
        rtn_val = FAIL;
      }
      break;

    case MOTOR_CALTYPE_MUL:
      val_ind_dbl = val_deg/motor_cal.deg_per_ind;
      break;

    default:
      PRINT_ERR_MSG("unknown calibration type");
      rtn_val = FAIL;
      break;
  }

  // Convert index value to an integer
  *val_ind = (int) floor(val_ind_dbl + 0.5);
  
  return rtn_val;
}

// ----------------------------------------------------
// Function: interp
//
// Purpose: interpolate value using x and y array data
// Note, data arrays must be Nx1 arrays of doubles. It
// if also assumed that the x_data arrays are sorted and
// that the values are unique. 
//
// Note, the assumptions should be checked in the check
// config routine as it will be too costly to do it in 
// the realtime loop. 
//
// Inputs:
//   x_data = array of x data
//   y_data = array of y data
//   x_val  = x_val at which to interpolate
//
// Outputs:
//   y_val  = pointer to y_val which is the result of
//             interpolating x_data and y_data at x_val
//
// -----------------------------------------------------
int interp(
    array_t x_data, 
    array_t y_data, 
    double x_val, 
    double *y_val
    )
{
  int i;
  int size; 
  int rtn_check;
  double max_x_val;
  double min_x_val;
  double x_next;
  double y_next;
  double x_lower;
  double y_lower;
  double x_upper;
  double y_upper;
  double a;
  double b;

  // Check array types
  if (x_data.type != DBL_ARRAY) {
    PRINT_ERR_MSG("x_data array must be of type DBL_ARRAY");
    return FAIL;
  } 
  if (y_data.type != DBL_ARRAY) {
    PRINT_ERR_MSG("y_data array must be of type DBL_ARRAY");
    return FAIL;
  } 

  // Check array shapes
  if (x_data.ncol != 1) {
    PRINT_ERR_MSG("x_data.ncol != 1");
    return FAIL;
  }
  if (y_data.ncol != 1) {
    PRINT_ERR_MSG("y_data.ncol != 1");
    return FAIL;
  }
  if (x_data.nrow != y_data.nrow) {
    PRINT_ERR_MSG("x_data.nrow != y_data.nrow");
    return FAIL;
  }
  size = x_data.nrow;

  // Check range
  rtn_check = get_array_val(x_data, 0, 0, &min_x_val);
  if (rtn_check == FAIL) {
    PRINT_ERR_MSG("get_array_val failed");
    return FAIL;
  }
  rtn_check = get_array_val(x_data, size-1, 0, &max_x_val);
  if (rtn_check == FAIL) {
    PRINT_ERR_MSG("get_array_val failed");
    return FAIL;
  }
  if (x_val < min_x_val) {
    PRINT_ERR_MSG("x_val < min value in x_data");
    return FAIL; 
  }
  if (x_val > max_x_val) {
    PRINT_ERR_MSG("x_val > max value in x_data");
    return FAIL;
  }

  // Find largest value in x data (x_lower) which is less than or equal 
  // to x_val and the smallest value in x data (x_upper) which is greater 
  // than x_val. Also, get the corresponding y values (y_lower, y_upper).
  
  // Initialize x_lower and y_lower
  x_lower = min_x_val;
  rtn_check = get_array_val(y_data, 0, 0, &y_lower);
  if (rtn_check == FAIL) {
    PRINT_ERR_MSG("get_array_val failed");
    return FAIL;
  }

  // Initialize x_upper and y_upper
  x_upper = x_lower;
  y_upper = y_lower;

  // Loop over x_data
  for (i=1; i<size; i++) {
    rtn_check = get_array_val(x_data, i, 0, &x_next);
    if (rtn_check == FAIL) {
      PRINT_ERR_MSG("get_array_val failed");
      return FAIL; 
    }
    rtn_check = get_array_val(y_data, i, 0, &y_next);
    if (rtn_check == FAIL) {
      PRINT_ERR_MSG("get_array_val failed");
      return FAIL; 
    }

    if (x_next <= x_val) {
      x_lower = x_next;
      y_lower = y_next;
    }
    else {
      x_upper = x_next;
      y_upper = y_next;
      break;
    }
  }

  // Get interpolated value
  if ((x_upper - x_lower) == 0.0) {
    PRINT_ERR_MSG("divide by zero, x_lower == x_upper");
    return FAIL;
  }
  a = (y_upper - y_lower)/(x_upper - x_lower);
  b = y_lower - a*x_lower; 
  *y_val = a*x_val + b;

  return SUCCESS; 
}

// -----------------------------------------------------
// Function: init_array
//
// Purpose: initializes an array structure w/ the given
// number of rows and columns
//
// -----------------------------------------------------
int init_array(array_t *array, int nrow, int ncol, int type)
{
  size_t size;

  // Check dimensions
  if (nrow <= 0) {
    PRINT_ERR_MSG("nrow <= 0");
    return FAIL;
  }
  if (ncol <= 0) {
    PRINT_ERR_MSG("ncol <= 0");
    return FAIL;
  }
  
  // Determine size based on type
  switch(type){
    
  case INT_ARRAY:
    size = sizeof(int);
    break;
      
  case FLT_ARRAY:
    size = sizeof(float);
    break;

  case DBL_ARRAY:
    size = sizeof(double);
    break;
    
  default:
    PRINT_ERR_MSG("unknown type");
    return FAIL;
    break;
  }

  // Allocate array
  array -> nrow = nrow;
  array -> ncol = ncol;
  array -> s0 = (array->ncol)*size;
  array -> s1 = size;
  array -> data = calloc((array->nrow)*(array->ncol), size);
  array -> type = type;

  if (array->data == NULL) {
    PRINT_ERR_MSG("unable to allocate memory");
    return FAIL;
  }
  else {
    return SUCCESS;
  }

}

// ------------------------------------------------------
// Function: free_array
//
// Purpose: free array memory
//
// ------------------------------------------------------
void free_array(array_t *array)
{
  if (array->data != NULL) {
    free(array -> data);
    array -> data = NULL;
    array -> nrow = 0;
    array -> ncol = 0;
    array -> s0 = 0;
    array -> s1 = 0;
    array -> type = EMPTY_ARRAY;
  }
}

// ----------------------------------------------------------------
// Function: print_config
//
// Purpose: prints configuration
//
// ----------------------------------------------------------------
void print_config(config_t config)
{
  int i;
  
  printf("\n              configuration\n");
  printf(" ------------------------------------------------\n");
  printf("  dev_name:            %s\n", config.dev_name);
  printf("  ain_subdev:          %d\n", config.ain_subdev);
  printf("  dio_subdev:          %d\n", config.dio_subdev);
  printf("  num_motor:           %d\n", config.num_motor);
  printf("  yaw_motor:           %d\n", config.yaw_motor);
  // Print dio clock
  printf("  dio_ckl:             {");
  for (i=0;i<config.num_motor;i++) {
    printf("%d", config.dio_clk[i]);
    if (i < config.num_motor-1) printf(", ");
  }
  printf("}\n");
  // Print dio direction
  printf("  dio_dir:             {");
  for (i=0;i<config.num_motor;i++) {
    printf("%d", config.dio_dir[i]);
    if (i < config.num_motor-1) printf(", ");
  }
  printf("}\n");
  // Print kine_map
  printf("  kine_map:            {");
  for (i=0;i<(config.num_motor-1);i++) {
    printf("%d", config.kine_map[i]);
    if (i < config.num_motor-2) printf(", ");
  }
  printf("}\n");
  printf("  dio_disable:         %d\n", config.dio_disable);
  printf("  yaw_ain:             %d\n", config.yaw_ain);
  printf("  yaw_ain_zero_dt:     %f\n", config.yaw_ain_zero_dt);
  printf("  yaw_ain_zero_num:    %d\n", config.yaw_ain_zero_num);
  printf("  yaw_volt2torq:       %f\n", config.yaw_volt2torq);
  printf("  yaw_inertia:         %f\n", config.yaw_inertia);
  printf("  yaw_ind2deg:         %f\n", config.yaw_ind2deg);
  printf("  yaw_torq_lim:        %f\n", config.yaw_torq_lim);
  printf("  yaw_torq_deadband:   %f\n", config.yaw_torq_deadband);
  printf("  yaw_filt_lpcut:      %f\n", config.yaw_filt_lpcut);
  printf("  yaw_filt_hpcut:      %f\n", config.yaw_filt_hpcut);
  printf("  yaw_damping:         %f\n", config.yaw_damping);
  printf("  dt:                  %f\n", NS2S*config.dt);
  printf("  integ_type:          ");
  if (config.integ_type == INTEG_RKUTTA) {
      printf("Runge-Kutta\n");
  }
  else if (config.integ_type == INTEG_EULER) {
      printf("Euler\n");
  }
  else {
      printf("unkown\n");
  }
  printf("  startup_t:           %f\n", config.startup_t);
  printf("  ff_flag:             ");
  if (config.ff_flag == FF_ON) {
      printf("FF_ON\n");
  }
  else if (config.ff_flag == FF_OFF) {
      printf("FF_OFF\n");
  }
  else {
      printf("unknown\n");
  }
  printf("  ctlr_flag:           ");
  if (config.ctlr_flag == CTLR_ON) {
      printf("CTLR_ON\n");
  }
  else if (config.ctlr_flag == CTLR_OFF) {
      printf("CTLR_OFF\n");
  }
  else {
      printf("unknown\n");
  }

  if (config.ctlr_flag == CTLR_ON) {

    printf("  ctlr_type:           ");
    if (config.ctlr_param.type == CTLR_TYPE_VEL) {
        printf("CTLR_TYPE_VEL\n");
    }
    else if (config.ctlr_param.type == CTLR_TYPE_POS) {
        printf("CTLR_TYPE_POS\n");
    }
    else {
        printf("unknown\n");
    }
    
    printf("  ctlr_pgian:          %1.2f\n", config.ctlr_param.pgain);
    printf("  ctlr_dgian:          %1.2f\n", config.ctlr_param.dgain);
    printf("  stroke amp:          %1.2f\n", config.kine_param.stroke_amp);
    printf("  rotation amp:        %1.2f\n", config.kine_param.rotation_amp);
    printf("  stroke_k:            %1.2f\n", config.kine_param.stroke_k);
    printf("  rotation_k:          %1.4f\n", config.kine_param.rotation_k); 
    printf("  wing beat period:    %1.2f\n", config.kine_param.period);
    printf("  kine type:           %s\n", KINE_TYPE_ID_2_NAME[config.kine_param.type]); 
    
    for (i=0; i<config.num_motor;i++) {
      printf("  motor[%d]\n", i);
      printf("   id:                 %s\n",MOTORID_2_NAME[config.motor_id_map[i]]); 
      if (config.motor_cal[i].type == MOTOR_CALTYPE_TBL) {
        printf("   cal type:           table\n");
      } 
      else if (config.motor_cal[i].type == MOTOR_CALTYPE_MUL) {
        printf("   cal type:           multiplication\n");
      }
    }
  }

  printf(" ------------------------------------------------\n");
  printf("\n");
  
  return;
}

// -----------------------------------------------------------------
// Function: print_err_msg
//
// Purpose: prints simple error message
// -----------------------------------------------------------------
void print_err_msg(const char *file, int line, const char *func, char *err_msg)
{
  fprintf(stderr, "%s:%d, %s, Error: %s\n",file, line, func, err_msg);
  return;
}

// -----------------------------------------------------------------
// Function: fflush_printf
//
// Purpose: print and flush stream
// -----------------------------------------------------------------
void fflush_printf(const char *format, ...)
{
  va_list ap;
  va_start(ap, format);
  vfprintf(stdout, format, ap);
  va_end(ap);
  fflush(stdout);
  return;
}


// Simple functions for getting constants to python ctypes interface
int define_max_motor(void) {return MAX_MOTOR;};
int define_max_dt(void) {return MAX_DT_NS;};
int define_min_dt(void) {return MIN_DT_NS;};
int define_clock_hi_ns(void) {return CLOCK_HI_NS;};
int define_integ_euler(void) {return INTEG_EULER;};
int define_integ_rkutta(void) {return INTEG_RKUTTA;};
int define_integ_unknown(void) {return INTEG_UNKNOWN;};
int define_empty_array(void) {return EMPTY_ARRAY;};
int define_int_array(void) {return INT_ARRAY;};
int define_flt_array(void) {return FLT_ARRAY;};
int define_dbl_array(void) {return DBL_ARRAY;};
int define_unknown_array(void) {return UNKNOWN_ARRAY;};
int define_success(void) {return SUCCESS;};
int define_fail(void) {return FAIL;};
int define_ff_on(void) {return FF_ON;};
int define_ff_off(void) {return FF_OFF;};
int define_ctlr_on(void) {return CTLR_ON;};
int define_ctlr_off(void) {return CTLR_OFF;};
int define_ctlr_type_vel(void) {return CTLR_TYPE_VEL;};
int define_ctlr_type_pos(void) {return CTLR_TYPE_POS;};
int define_motor_caltype_tbl(void) {return MOTOR_CALTYPE_TBL;};
int define_motor_caltype_mul(void) {return MOTOR_CALTYPE_MUL;};

int stroke_0_id(void) {return STROKE_0_ID;};
int stroke_1_id(void) {return STROKE_1_ID;};
int rotation_0_id(void) {return ROTATION_0_ID;};
int rotation_1_id(void) {return ROTATION_1_ID;};
int deviation_0_id(void) {return DEVIATION_0_ID;};
int deviation_1_id(void) {return DEVIATION_1_ID;};
int yaw_id(void) {return YAW_ID;};

int diff_aoa_id(void) {return DIFF_AOA_ID;};
int diff_dev_id(void) {return DIFF_DEV_ID;};
