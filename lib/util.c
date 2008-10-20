// ----------------------------------------------------------------
// File: util.h
//
// Puropse: contains vairous utility functions used by the yawff
// library.
//
// Functions:
// 
//   integrator         = 2nd order integrator
//   dynamics_func      = function for yaw dynamics
//   init_array         = initializes array object
//   free_array         = frees array object
//   set_array_val      = sets array element value
//   get_array_val      = gets array  element value
//   lowpass_filt1      = firat order lowpass filter
//   print_config       = prints configuration structure
//   print_err_msg      = prints error messages
//   get_max_motor      = returns maximum allowed number of motors
//   get_max_dt         = returns maximum allowed real-time period
//   get_min_dt         = returns minimum allowed real-time period
//   get_clock_hi_ns    = returns the clock high time in ns
//
// Author: Will Dickson
//
// ----------------------------------------------------------------
#include "yawff.h"

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
// Purpose: 2nd order integrator for turning yaw torque into 
// position. Currently can select bewteen 1st order Euler method 
// and 4th order Runge-Kutta 
//
// Arguments:
//   state_curr  = dynamic state (position + velocity) at current 
//                 time step
//   state_next  = dynamic state ate next time step (output) 
//   force       = external force or torque 
//   mass        = mass or moment of inertia
//   damping     = damping constant
//   dt          = time step
//   method      = integration method (INTEG_EULER or INTEG_RKUTTA)
//
// Return:
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
    
    print_err_msg(__FILE__,__LINE__,"unknown integration method");
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


// ----------------------------------------------------------------
// Function: get_array_val
//
// Puropse: gets value of array at given row and column
//
// -----------------------------------------------------------------
int get_array_val(array_t *array, int row, int col, void *val)
{
  int s0, s1;
  int *iptr;
  float *fptr;
  int rtn_val = SUCCESS;

  // Check row and col ranges
  if ((row < 0) || (row >= (array -> nrow))) {
    print_err_msg(__FILE__,__LINE__,"get_array_val, row out of range");
    return FAIL;
  }
  if ((col < 0) || (col >= array ->ncol)) {
    print_err_msg(__FILE__,__LINE__,"get_array_val, col out of range");
    return FAIL;
  }

  // Get array value
  s0 = array -> s0;
  s1 = array -> s1;

  switch(array->type) {
    
  case INT_ARRAY:
    iptr = val;
    *iptr = *((int*)( (array -> data) + row*s0 + col*s1));
    break;

  case FLT_ARRAY:
    fptr = val;
    *fptr = *((float*)( (array -> data) + row*s0 + col*s1));
    break;

  default:
    print_err_msg(__FILE__,__LINE__,"get_array_val, unknown array type");
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
int set_array_val(array_t *array, int row, int col, void *val)
{
  int s0, s1;
  int *iptr;
  float *fptr;
  int rtn_val = SUCCESS;

  // Check row and col ranges
  if ((row < 0) || (row >= (array -> nrow))) {
    print_err_msg(__FILE__,__LINE__,"get_array_val, row out of range");
    return FAIL;
  }
  if ((col < 0) || (col >= array ->ncol)) {
    print_err_msg(__FILE__,__LINE__,"get_array_val, col out of range");
    return FAIL;
  }
  
  // Set array value
  s0 = array -> s0;
  s1 = array -> s1;

  switch(array->type) {

  case INT_ARRAY:
    iptr = val;
    *((int*)((array -> data) + row*s0 + col*s1)) = *iptr;
    break;

  case FLT_ARRAY:
    fptr = val;
    *((float*)((array -> data) + row*s0 + col*s1)) = *fptr;    
    break;

  default:
    print_err_msg(__FILE__,__LINE__,"set_array_val, unknown array type");
    rtn_val = FAIL;
    break;
    
  }
  return rtn_val;
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
    print_err_msg(__FILE__,__LINE__,"init_array, nrow <= 0");
    return FAIL;
  }
  if (ncol <= 0) {
    print_err_msg(__FILE__,__LINE__,"init_array, ncol <= 0");
    return FAIL;
  }
  
  // Determine size based om type
  switch(type){
    
  case INT_ARRAY:
    size = sizeof(int);
    break;
      
  case FLT_ARRAY:
    size = sizeof(float);
    break;
    
  default:
    print_err_msg(__FILE__,__LINE__,"init_array, unknown type");
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
    print_err_msg(__FILE__,__LINE__,"init_array unable to allocate memory");
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
  printf(" ------------------------------------------\n");
  printf("  dev_name:       %s\n", config.dev_name);
  printf("  ain_subdev:     %d\n", config.ain_subdev);
  printf("  dio_subdev:     %d\n", config.dio_subdev);
  printf("  num_motor:      %d\n", config.num_motor);
  printf("  yaw_motor:      %d\n", config.yaw_motor);
  // Print dio clock
  printf("  dio_ckl:        {");
  for (i=0;i<config.num_motor;i++) {
    printf("%d", config.dio_clk[i]);
    if (i < config.num_motor-1) printf(", ");
  }
  printf("}\n");
  // Print dio direction
  printf("  dio_dir:        {");
  for (i=0;i<config.num_motor;i++) {
    printf("%d", config.dio_dir[i]);
    if (i < config.num_motor-1) printf(", ");
  }
  printf("}\n");
  // Print kine_map
  printf("  kine_map:       {");
  for (i=0;i<(config.num_motor-1);i++) {
    printf("%d", config.kine_map[i]);
    if (i < config.num_motor-2) printf(", ");
  }
  printf("}\n");
  printf("  yaw_ain:        %d\n", config.yaw_ain);
  printf("  yaw_volt2torq:  %f\n", config.yaw_volt2torq);
  printf("  yaw_inertia:    %f\n", config.yaw_inertia);
  printf("  yaw_ind2deg:    %f\n", config.yaw_ind2deg);
  printf("  yaw_torq_lim:   %f\n", config.yaw_torq_lim);
  printf("  yaw_filt_cut:   %f\n", config.yaw_filt_cut);
  printf("  dt:             %d\n", config.dt);
  printf(" ------------------------------------------\n");
  printf("\n");
  
  return;
}

// -----------------------------------------------------------------
// Function: print_err_msg
//
// Purpose: prints simple error message
// -----------------------------------------------------------------
void print_err_msg(char *file, int line, char *err_msg)
{
  fprintf(stderr, "%s:%d Error, %s\n",file, line, err_msg);
  return;
}

// Simple functions for getting constants to python ctypes interface
int get_max_motor(void) {return MAX_MOTOR;};
int get_max_dt(void) {return MAX_DT_NS;};
int get_min_dt(void) {return MIN_DT_NS;};
int get_clock_hi_ns(void) {return CLOCK_HI_NS;};
