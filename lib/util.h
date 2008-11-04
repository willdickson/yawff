#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "yawff.h"

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
