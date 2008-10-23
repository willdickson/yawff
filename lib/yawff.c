// -------------------------------------------------------------------
// File: yawff.c
//
// Purpose: 
// 
// Author: Will Dickson 09/30/2008
// -------------------------------------------------------------------
#include "yawff.h"

// Structure for arguments pass to real time thread
typedef struct {
  array_t *kine;
  config_t *config;
  data_t *data;
} thread_args_t;

// RT-task parameters
const int PRIORITY=1;
const int STACK_SIZE=4096;
const int MSG_SIZE=0;

// Function prototypes
static void *rt_handler(void *args);
int init_comedi(void);
int init_rtai(void);

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
int yawff(
	  array_t kine, 
	  config_t config, 
	  data_t data, 
	  int *end_pos
	  )
{

  int rt_thread;
  void *sighandler = NULL;
  int rtn_flag = SUCCESS;
  thread_args_t thread_args;
  int i;
  float val;

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

  // Initialize comedi device

  // Take zero reading from yaw torque sensor
  
  // Pack arguments to thread
  thread_args.kine = &kine;
  thread_args.config = &config;
  thread_args.data = &data;
  
  // Start motor thread
  fflush_printf("starting rt_thread\n");
  rt_thread = rt_thread_create(rt_handler, ((void *)&thread_args), STACK_SIZE);
  

  // Print information 

 
  // Clean up
  rt_thread_join(rt_thread);
  fflush_printf("rt_thread joined\n");

  // Restore old SIGINT handler
  sighandler = signal(SIGINT,sighandler);
  if (sighandler == SIG_ERR) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"restoring signal handler failed");
    rtn_flag = FAIL;
  }
  
  ///////////////////////////////////////////
  // Test reading data out of data array
  //for (i=0; i<data.t.nrow; i++) {
  //  get_array_val(data.t,i,0,&val);
  //  printf("%f\n", val);
  //}
  //////////////////////////////////////////

  // Temporary
  return rtn_flag;
}

// ------------------------------------------------------------------
// Function: rt_handler
//
// Purpose: Realtime thread for 
//
// ------------------------------------------------------------------
static void *rt_handler(void *args)
{
  RT_TASK *rt_task;
  thread_args_t *thread_args;
  array_t kine;
  config_t config;
  data_t data;
  int i;
  float val;

  // Unpack arguments passed to thread
  thread_args = (thread_args_t *) args;
  kine = *(thread_args -> kine);
  config = *(thread_args -> config);
  data = *(thread_args -> data);

  // Initialize rt_task
  fflush_printf("Initializing rt_task \n");
  rt_allow_nonroot_hrt();
  rt_task = rt_task_init_schmod(nam2num("MOTOR"),0, 0, 0, SCHED_FIFO, 0xF); 
  if (!rt_task) {          
    print_err_msg(__FILE__,__LINE__,__FUNCTION__, "cannot initialize rt task\n");
    end = 1;
    return 0;
  }

  
  ///////////////////////////////////////
  // Test copying data into data array
  //for (i=0; i<data.t.nrow; i++) {
  //  val = 10.0*( (float) i);
  //  set_array_val(data.t,i,0,&val);
  //}
  ///////////////////////////////////////


  // Clean up
  fflush_printf("deleting rt_task \n");
  rt_task_delete(rt_task);

  return 0;
}


// ------------------------------------------------------------------
// Function: sigint_func
//
// ------------------------------------------------------------------
void sigint_func(int sig) {
  end = 1;
  return;
}



