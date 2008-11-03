// ---------------------------------------------------------------
// yawff.h
//
// Purpose: Contains preprocessor directives, declarations and 
// functions definitions for the yawff library.
// 
// Author: Will Dickson
// --------------------------------------------------------------
#ifndef INC_YAWFF_H_
#define INC_YAWFF_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_comedi.h>
#include <signal.h>

#define SUCCESS 0
#define FAIL -1
#define DIO_HI 1
#define DIO_LO 0
#define MAX_MOTOR 10
#define MAX_DIO 24
#define MAX_AIN 15
#define ERR_SZ 200
#define MAX_DT_NS 10000000 // 100 Hz 
#define MIN_DT_NS 40000    // 25 kHz
#define CLOCK_HI_NS 40000  
#define MIN_TORQ_LIM 0.0   // Nm
#define MAX_TORQ_LIM 0.5   // Nm
#define NS2S 1.0e-9        // Convert nanoseconds to seconds 
#define INTEG_EULER 0      // Const for integration by Euler method 
#define INTEG_RKUTTA 1     // Const for integration by Runge-Kutta
#define INTEG_UNKNOWN 2    // Const for unknown integration method 
                           //(used for unit testing)

#define EMPTY_ARRAY 0      // Indicates array type is empty
#define INT_ARRAY 1        // Indicates array type ingeter
#define FLT_ARRAY 2        // Indicates array type float
#define UNKNOWN_ARRAY 3    // Indicates array of unkown type
                           //(use for unit testing)

#define AIN_ZERO_DT 0.01   // Sample time interval for zeroing analog input 
#define AIN_ZERO_NUM 50    // Number of samples to acquire when zeroing analog input
#define AIN_RANGE 0        // Analog input range
#define AIN_AREF AREF_GROUND // Analog input reference


// Structure for configuration
typedef struct {
  char *dev_name;          // Comedi device name  
  unsigned int ain_subdev; // Analog input subdevice
  int dio_subdev;          // Digital IO subdevice
  int num_motor;           // Total number of motors
  int yaw_motor;           // Yaw motor number
  int dio_clk[MAX_MOTOR];  // DIO clock pins
  int dio_dir[MAX_MOTOR];  // DIO direction pins
  int kine_map[MAX_MOTOR]; // Map from kinematics to motors
  unsigned int yaw_ain;    // Analog input channel for yaw torque
  float yaw_volt2torq;     // Voltage to torque conversion
  float yaw_inertia;       // Moment of inertia about yaw axis
  float yaw_ind2deg;       // Index to degree conversion for yaw
  float yaw_torq_lim;      // Yaw torque limit (Nm)
  float yaw_filt_cut;      // Yaw torque lowpass filter cutoff (Hz)
  float yaw_damping;       // Damping constant for yaw axis
  int dt;                  // Realtime loop timestep (ns)
} config_t;

// Structure for numpy array
typedef struct {
  void *data;
  int nrow;
  int ncol; 
  int s0; 
  int s1;
  int type;
} array_t;

// Structure for dynamic state
typedef struct {
  float pos;
  float vel;
} state_t;

// Structure for return data
typedef struct {
  array_t t;
  array_t pos;
  array_t vel;
  array_t torq;
} data_t;

// Structure for comedi device information 
typedef struct {
  void *device;
  comedi_krange krange;
  int maxdata;
} comedi_info_t;

// Structure for torque data
typedef struct {
  float zero;
  float last;
} torq_info_t;


// Prototypes for functions in yawff.c
extern int yawff(array_t kine, config_t config, data_t data, int *end_pos); 
extern int rt_cleanup(int level, comedi_info_t comedi_info, RT_TASK *rt_task);
extern int init_comedi(comedi_info_t *comedi_info, config_t config);
extern int get_ain_zero(comedi_info_t comedi_info,  config_t config, float *ain_zero);
extern int get_torq_zero(comedi_info_t comedi_info, config_t config, float *torq_zero);
extern int get_ain(comedi_info_t comedi_info, config_t config, float *ain);
extern int get_torq(comedi_info_t comedi_info, config_t config, float *torq);
extern int ain_to_phys(lsampl_t data, comedi_info_t comedi_info, float *volts);
extern int update_state(state_t *state, torq_info_t *torq_info, comedi_info_t comedi_info, config_t config);
extern void init_ind(int motor_ind[][2], config_t config);
extern int update_ind(int motor_ind[][2], array_t kine, int kine_ind, state_t *state, config_t config);
extern int update_motor(int motor_ind[][2], comedi_info_t comedi_info, config_t config);
extern int update_data(data_t data, int ind, float t, state_t *state, torq_info_t torq_info); 
extern int set_clks_lo(comedi_info_t comedi_info, config_t config);

// Prototypes for functions if util.c
extern int integrator(state_t state_curr, state_t *state_next, float force, 
		      float mass, float damping, float dt, int method);
extern float lowpass_filt1(float x,float y, float f_cut, float dt);
extern int init_array(array_t *array, int nrow, int ncol, int type);
extern void free_array(array_t *array);
extern int get_array_val(array_t array, int row, int col, void *val);
extern int set_array_val(array_t array, int row, int col, void *val);
extern void print_config(config_t config);
extern void print_err_msg(const char *file, int line, const char *func, char *err_msg);
extern void fflush_printf(const char *format, ...);
extern int get_max_motor(void);
extern int get_max_dt(void);
extern int get_min_dt(void);
extern int get_clock_hi_ns(void);

// Prototype for functions in check.c
extern int check_yawff_input(array_t kine, config_t config, data_t data);
extern int check_config(config_t config);
extern int check_ranges(config_t config);
extern int check_clkdir(config_t config);
extern int check_kine_map(config_t config);
extern int check_kine(array_t kine);
extern int check_kine_compat(config_t config, array_t kine);
extern int check_array(array_t array);
extern int check_data(data_t data);
extern int check_data_compat(array_t kine, data_t data);

#endif // INC_YAWFF_H_

