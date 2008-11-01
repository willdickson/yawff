// -------------------------------------------------------------------
// File: yawff.c
//
// Purpose: 
// 
// Author: Will Dickson 09/30/2008
// -------------------------------------------------------------------
#include "yawff.h"
#include <unistd.h>
#include <time.h>

// RT-task parameters
#define PRIORITY 1
#define STACK_SIZE 4096
#define MSG_SIZE 0

// RT-task cleanup levels
#define RT_CLEANUP_LEVEL_1 1
#define RT_CLEANUP_LEVEL_2 2
#define RT_CLEANUP_ALL 2

// Structure for arguments pass to real time thread
typedef struct {
  array_t *kine;
  config_t *config;
  data_t *data;
} thread_args_t;

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
 
// Function prototypes
static void *rt_handler(void *args);
int rt_cleanup(int level, comedi_info_t comedi_info, RT_TASK *rt_task);
int init_comedi(comedi_info_t *comedi_info, config_t config);
int get_ain_zero(comedi_info_t comedi_info,  config_t config, float *ain_zero);
int get_torq_zero(comedi_info_t comedi_info, config_t config, float *torq_zero);
int get_ain(comedi_info_t comedi_info, config_t config, float *ain);
int get_torq(comedi_info_t comedi_info, config_t config, float *torq);
int ain_to_phys(lsampl_t data, comedi_info_t comedi_info, float *volts);
int update_state(state_t *state, torq_info_t *torq_info, comedi_info_t comedi_info, config_t config);
void sigint_func(int sig);

// Global variables
volatile int end = 0;

// -------------------------------------------------------------------
// Function: yawff
//
// Puropse: Force-feedback yaw turn. 
//
// Arguments:
//   
//  kine     = wing kinematics function
//  config   = system configuration data
//  data     = structure of return data arrays
//  end_pod  = pointer to final position in motor ind
//  
// -------------------------------------------------------------------
int yawff(array_t kine, config_t config, data_t data, int *end_pos)
{

  RT_TASK *yawff_task;
  int rt_thread;
  void *sighandler = NULL;
  int rtn_flag = SUCCESS;
  thread_args_t thread_args;

  // Startup method 
  printf("\n");
  printf("                  Starting yawff \n");
  printf("=======================================================\n");

  // Check inputs
  if (check_yawff_input(kine,config,data) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"bad input data");
    return FAIL;
  }
  print_config(config);

  // Setup SIGINT handler
  sighandler = signal(SIGINT,sigint_func);
  if (sighandler == SIG_ERR) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__, "assigning SIGINT handler");
    return FAIL;
  }
  
  //Initialize RT task
  fflush_printf("initializing yawff_task\n");
  rt_allow_nonroot_hrt();
  yawff_task = rt_task_init(nam2num("YAWFF"),PRIORITY,STACK_SIZE,MSG_SIZE);
  if (!yawff_task) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"error initializing yawff_task\n");
    return FAIL;
  }
  rt_set_oneshot_mode();
  start_rt_timer(0);

  // Assign arguments to thread
  thread_args.kine = &kine;
  thread_args.config = &config;
  thread_args.data = &data;
  
  // Start motor thread
  fflush_printf("starting rt_thread\n");
  rt_thread = rt_thread_create(rt_handler, ((void *)&thread_args), STACK_SIZE);
  
  // Run time display
  do {
    sleep(4);
  } while (!end);

  // Clean up
  rt_thread_join(rt_thread);
  fflush_printf("rt_thread joined\n");
  stop_rt_timer();
  rt_task_delete(yawff_task);
  fflush_printf("yawff_task deleted\n");

  // Restore old SIGINT handler
  sighandler = signal(SIGINT,sighandler);
  if (sighandler == SIG_ERR) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"restoring signal handler failed");
    rtn_flag = FAIL;
  }
  
  // Temporary
  return rtn_flag;
}

// ------------------------------------------------------------------
// Function: rt_handler
//
// Purpose: Realtime thread function. Description ... 
//
// ------------------------------------------------------------------
static void *rt_handler(void *args)
{
  RT_TASK *rt_task=NULL;
  thread_args_t *thread_args=NULL;
  comedi_info_t comedi_info;
  array_t kine;
  config_t config;
  data_t data;
  RTIME now_ns;
  torq_info_t torq_info;
  state_t state[2]; // state[0] = previous state, state[1] = current state
  float runtime;
  int ind[MAX_MOTOR];
  int i,j;

  // Unpack arguments passed to thread
  thread_args = (thread_args_t *) args;
  kine = *(thread_args -> kine);
  config = *(thread_args -> config);
  data = *(thread_args -> data);

  // Initialize comedi device
  if (init_comedi(&comedi_info, config) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to initialize comedi device");
    end = 1;
    return 0;
  }

  // Find yaw torque zero
  if (get_torq_zero(comedi_info, config, &torq_info.zero) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"failed to get ain zero");
    // Error, clean up and exit
    if (rt_cleanup(RT_CLEANUP_LEVEL_1,comedi_info,rt_task) != SUCCESS) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"rt_cleanup failed");
    }
    end = 1;
    return 0;
  }  

  // Initialize rt_task
  fflush_printf("Initializing rt_task \n");
  rt_task = rt_task_init_schmod(nam2num("MOTOR"),0, 0, 0, SCHED_FIFO, 0xF);
  if (!rt_task) {          
    print_err_msg(__FILE__,__LINE__,__FUNCTION__, "cannot initialize rt task");
    // Error, clean up and exit
    if (rt_cleanup(RT_CLEANUP_LEVEL_1,comedi_info,rt_task) != SUCCESS) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"rt_cleanup failed");
    }
    end = 1;
    return 0;
  }
  rt_task_use_fpu(rt_task,1); // Enable use of floating point math
  
  // Setup periodic timer
  fflush_printf("setting up periodic timer\n");

  // Initialize dynamic state
  for (i=0; i<2; i++) {
    state[i].pos = 0.0;
    state[i].vel = 0.0;
  }  
  // Go to hard real-time
  runtime = ((float) config.dt)*((float) kine.nrow)*NS2S;
  fflush_printf("going to hard real-time, T = %f\n", runtime);
  mlockall(MCL_CURRENT|MCL_FUTURE);
  rt_make_hard_real_time();

  // Loop over kinematic indices
  for (i=0; i<kine.nrow; i++) {

    now_ns = rt_get_time_ns();

    // DEBUG ///////////////////////////////////////////////////////
    comedi_dio_write(comedi_info.device,config.dio_subdev,0,DIO_HI);
    ////////////////////////////////////////////////////////////////
      
    // Update dynamic state
    if (update_state(state,&torq_info,comedi_info,config) != SUCCESS) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"updating dynamic state failed");
      goto RT_LOOP_EXIT;
    }
    
    // Loop over wing motors and update motor positions
    for (j=0; j<kine.ncol;j++) {
      if (get_array_val(kine,i,0,&ind[j]) != SUCCESS) {
	print_err_msg(__FILE__,__LINE__,__FUNCTION__, "error accessing kine");
	goto RT_LOOP_EXIT;
      }
    } // End for j

    // Update yaw motor position
    state[0] = state[1];
    

    // Sleep for CLOCK_HI_NS and then set clock lines low
    rt_sleep_until(nano2count(now_ns + (RTIME) CLOCK_HI_NS));

    // DEBUG ///////////////////////////////////////////////////////
    comedi_dio_write(comedi_info.device,config.dio_subdev,0,DIO_LO);
    ////////////////////////////////////////////////////////////////    

    // Check if end has been set to 1 by SIGINT
    if (end == 1) {
      fflush_printf("SIGINT - exiting real-time\n");
      goto RT_LOOP_EXIT;
    }

    // Sleep until next period
    rt_sleep_until(nano2count(now_ns + config.dt));

  } // End for i

 RT_LOOP_EXIT:
  // Leave realtime
  rt_make_soft_real_time();
  munlockall();
  fflush_printf("leaving hard real-time\n");

  // Clean up
  if (rt_cleanup(RT_CLEANUP_ALL, comedi_info, rt_task)!=SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"rt_cleanup failed");
  }
  end = 1;

  return 0;
}

// ----------------------------------------------------------------------
// Function: update_state
//
// Purpose: Get reading from torque sensor and update dynamic state.
//
// ---------------------------------------------------------------------- 
int update_state(state_t *state, 
		 torq_info_t *torq_info,
		 comedi_info_t comedi_info, 
		 config_t config
		 )
{

  float dt;
  float torq_raw;
  float torq_filt;
  int rval;

  // Get time step in secs
  dt = config.dt*NS2S;
  
  // Get data from  torque sensor and zero 
  if (get_torq(comedi_info, config, &torq_raw) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__, "error reading torque");
    return FAIL;
  }
  torq_raw = torq_raw-(torq_info->zero);

  // Zero and filter torque
  torq_filt = lowpass_filt1(torq_raw,torq_info->last,config.yaw_filt_cut,dt);
  torq_info -> last = torq_filt;

  // Set previous state to current state
  state[0] = state[1];
  
  // Integrate one time step
  rval = integrator(state[1],
		    &state[1],
		    torq_filt,
		    config.yaw_inertia, 
		    config.yaw_damping, 
		    dt,
		    INTEG_RKUTTA);
  if (rval != SUCCESS ) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"integrator failed");
    return FAIL;
  }
 
  return SUCCESS;
}

// ----------------------------------------------------------------------
// Function: get_torq_zero
//
// Purpose: Determines the zero value (Nm) for yaw torque sensor. 
//
// ----------------------------------------------------------------------
int get_torq_zero(comedi_info_t comedi_info, config_t config, float *torq_zero)
{
  float ain_zero;

  // Get analog input zero
  if (get_ain_zero(comedi_info, config, &ain_zero) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to get ain zero");
    return FAIL;
  }

  // Convert to torque
  *torq_zero = ain_zero*config.yaw_volt2torq;
  
  fflush_printf("torque zero: %f(Nm)\n", *torq_zero);

  return SUCCESS;
}

// -----------------------------------------------------------------------
// Function: get_ain_zero
//
// Purpose: Determines the zero value in volts for the yaw torque
// analog input.
//
// -----------------------------------------------------------------------
int get_ain_zero(comedi_info_t comedi_info, config_t config, float *ain_zero)
{
  int i;
  float ain;
  int ret_flag = SUCCESS;
  char err_msg[ERR_SZ];
  struct timespec sleep_req;

  fflush_printf("finding ain zero, T = %1.3f(s) \n", AIN_ZERO_DT*AIN_ZERO_NUM);

  // Set sleep timespec
  sleep_req.tv_sec = (time_t) AIN_ZERO_DT;
  sleep_req.tv_nsec = 1.0e9*(AIN_ZERO_DT - (time_t) AIN_ZERO_DT);

  // Get samples and find mean
  *ain_zero = 0;
  for (i=0; i<AIN_ZERO_NUM; i++) {

    if (get_ain(comedi_info, config, &ain) != SUCCESS) {
      snprintf(err_msg, ERR_SZ, "unable to read ain, i = %d", i);
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,err_msg);
      ret_flag = FAIL;
      break;
    }

    // Compute running mean
    *ain_zero = (((float) i)/((float) i+1))*(*ain_zero)+(1.0/((float) i+1))*ain;
    
    // Sleep for AIN_SLEEP_DT seconds
    nanosleep(&sleep_req, NULL);
  }
  fflush_printf("ain_zero: %f(V)\n", *ain_zero);

  return ret_flag;
}

// ------------------------------------------------------------------
// Function: get_ain
//
// Prupose: Read yaw torq analog input
//
// ------------------------------------------------------------------
int get_ain(comedi_info_t comedi_info, config_t config, float *ain)
{
  int rval;
  lsampl_t ain_lsampl; 

  // Read value from daq card
  rval = comedi_data_read(comedi_info.device,
			  config.ain_subdev,
			  config.yaw_ain,
			  AIN_RANGE,
			  AIN_AREF,
			  &ain_lsampl);
  if (rval!=1) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"comedi_data_read failed");
    return FAIL;
  }
  
  // Convert integer analog input value to volts
  if (ain_to_phys(ain_lsampl, comedi_info, ain) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"ain_to_phys failed");
    return FAIL;
  }
  return SUCCESS;
}
// ------------------------------------------------------------------
// Function: get_torq
//
// Purpose: Read yaw torque from sensor
//
// ------------------------------------------------------------------
int get_torq(comedi_info_t comedi_info, config_t config, float *torq)
{

  float ain;
  
  if (get_ain(comedi_info, config, &ain) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to read analog input");
    return FAIL;
  }

  *torq = ain*config.yaw_volt2torq;

  return SUCCESS;
}

// ------------------------------------------------------------------
// Function: init_comedi
//
// Purpose: Opens comedi device and sets clock and direction dio 
// lines to output.
//
// ------------------------------------------------------------------
int init_comedi(comedi_info_t *comedi_info, config_t config)
{
  int i;
  int rval;
  char err_msg[ERR_SZ];
  int ret_flag = SUCCESS;
  
  // Open comedi device
  fflush_printf("opening comedi device\n");
  comedi_info->device = comedi_open(config.dev_name);
  if (comedi_info->device == NULL) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to open comedi device");
    return FAIL;
  }

  // Configure comedi dio 
  for (i=0; i<config.num_motor; i++) {    
    // Set clock lines to output
    rval = comedi_dio_config(comedi_info->device, 
			     config.dio_subdev, 
			     config.dio_clk[i],
			     COMEDI_OUTPUT);
    if (rval != 1 ) {
      snprintf(err_msg, ERR_SZ, "unable to configure dio_clk[%d]", i);
      print_err_msg(__FILE__,__LINE__,__FUNCTION__, err_msg);
      ret_flag = FAIL;
    }
    // Set direction lines to output 
    rval = comedi_dio_config(comedi_info->device, 
			     config.dio_subdev, 
			     config.dio_dir[i],
			     COMEDI_OUTPUT);
    if (rval != 1) {
      snprintf(err_msg, ERR_SZ, "unable to configure dio_dir[%d]", i);
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,err_msg);
      ret_flag = FAIL;
    }
  } // End for i
 
  // Get max data and krange for conversion to physical units
  comedi_info->maxdata = comedi_get_maxdata(comedi_info -> device, 
					    config.ain_subdev, 
					    config.yaw_ain);
  rval = comedi_get_krange(comedi_info->device, 
			   config.ain_subdev, 
			   config.yaw_ain, 
			   AIN_RANGE, 
			   &(comedi_info->krange));
  if (rval < 0) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to get krange");
    return FAIL;
  }
  return ret_flag;
}

// ------------------------------------------------------------------
// Function: rt_cleanup
//
// Purpose: Cleanup function for real-time task. Cleans up based on 
// cleanup level. 
// 
// level 1 => closes comedi device
// level 2 => deletes rt_task and then closes comedi device.
//
// ------------------------------------------------------------------
int rt_cleanup(int level, comedi_info_t comedi_info, RT_TASK *rt_task)
{
  
  int ret_flag = SUCCESS;

  fflush_printf("starting rt_cleanup: level=%d\n", level);
  
  switch (level) {
    
  case RT_CLEANUP_LEVEL_2:
    fflush_printf("  %d: deleting rt_task\n", RT_CLEANUP_LEVEL_2);
    if (rt_task_delete(rt_task) != 0) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to delete rt_task");
      ret_flag = FAIL;
    }
    
  case RT_CLEANUP_LEVEL_1:
   
    fflush_printf("  %d: closing comedi device\n", RT_CLEANUP_LEVEL_1);
    if (comedi_close(comedi_info.device)!=0) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to close comedi device");
      ret_flag = FAIL;
    }

  default:
    break;
  }

  fflush_printf("rt_cleanup complete\n");

  return ret_flag;
}


// -------------------------------------------------------------------
// Function: ain_to_phys
//
// Purpose: Converts analog input reading to physical units (volts)
//
// -------------------------------------------------------------------
int ain_to_phys(lsampl_t data, comedi_info_t comedi_info, float *volts)
{
  float max_rng;
  float min_rng;
  
  // Check if maxdata is zero
  if (comedi_info.maxdata==0) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"maxdata == 0"); 
    return FAIL;
  }

  // Get max and min ranges, Is this correct ??
  max_rng = ((float) (comedi_info.krange.max))*1.0e-6;
  min_rng = ((float) (comedi_info.krange.min))*1.0e-6;
  
  // Convert integer data to float voltages
  *volts = (float) data;
  *volts /= (float) comedi_info.maxdata;
  *volts *= (max_rng - min_rng);
  *volts += min_rng;

  return SUCCESS;
}

// ------------------------------------------------------------------
// Function: sigint_func
//
// ------------------------------------------------------------------
void sigint_func(int sig) {
  end = 1;
  return;
}



