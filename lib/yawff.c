// -------------------------------------------------------------------
// File: yawff.c
//
// Purpose: 
// 
// Author: Will Dickson 09/30/2008
// -------------------------------------------------------------------
#include "yawff.h"
#include <unistd.h>

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

// Function prototypes
static void *rt_handler(void *args);
int rt_cleanup(int level, void *comedi_device, RT_TASK *rt_task);
int init_comedi(void **comedi_device, config_t config);

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
  
  // Pack arguments to thread
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
// Purpose: Realtime thread for 
//
// ------------------------------------------------------------------
static void *rt_handler(void *args)
{
  RT_TASK *rt_task=NULL;
  thread_args_t *thread_args=NULL;
  void *comedi_device=NULL;
  array_t kine;
  config_t config;
  data_t data;
   
  int ind[MAX_MOTOR];
  int i,j;

  // Unpack arguments passed to thread
  thread_args = (thread_args_t *) args;
  kine = *(thread_args -> kine);
  config = *(thread_args -> config);
  data = *(thread_args -> data);


  // Initialize comedi device
  if (init_comedi(&comedi_device, config) != SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to initialize comedi device");
    end = 1;
    return 0;
  }

  // Zero torque input

  // Initialize rt_task
  fflush_printf("Initializing rt_task \n");
  rt_allow_nonroot_hrt();
  rt_task = rt_task_init_schmod(nam2num("MOTOR"),0, 0, 0, SCHED_FIFO, 0xF); 
  if (!rt_task) {          
    print_err_msg(__FILE__,__LINE__,__FUNCTION__, "cannot initialize rt task");
    // clean up and exit
    if (rt_cleanup(RT_CLEANUP_LEVEL_1,comedi_device,rt_task) != SUCCESS) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"rt_cleanup failed");
    }
    end = 1;
    return 0;
  }

  // Loop over motor indices
  for (i=0; i<kine.nrow; i++) {
    //printf("%d: ", i);
    for (j=0; j<kine.ncol;j++) {
      if (get_array_val(kine,i,0,&ind[j]) != SUCCESS) {
	print_err_msg(__FILE__,__LINE__,__FUNCTION__, "error accessing kine");
      }
      //printf("%d, ", ind[j]);
    }
    //printf("\n");
  }

  // Clean up
  if (rt_cleanup(RT_CLEANUP_ALL, comedi_device, rt_task)!=SUCCESS) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"rt_cleanup failed");
  }
  end = 1;

  return 0;
}


// ------------------------------------------------------------------
// Function: init_comedi
//
// Purpose: Opens comedi device and sets clock and direction dio 
// lines to output.
//
// ------------------------------------------------------------------
int init_comedi(void **comedi_device, config_t config)
{
  int i;
  int rval;
  char err_msg[ERR_SZ];
  int ret_flag = SUCCESS;
  
  // Open comedi device
  fflush_printf("opening comedi device\n");
  *comedi_device = comedi_open(config.dev_name);
  if (comedi_device == NULL) {
    print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to open comedi device");
    return FAIL;
  }

  // Configure comedi dio 
  for (i=0; i<config.num_motor; i++) {    
    // Set clock lines to output
    rval = comedi_dio_config(*comedi_device, 
				config.dio_subdev, 
				config.dio_clk[i],
				COMEDI_OUTPUT);
    if (rval != 1 ) {
      snprintf(err_msg, ERR_SZ, "unable to configure dio_clk[%d]", i);
      print_err_msg(__FILE__,__LINE__,__FUNCTION__, err_msg);
      ret_flag = FAIL;
    }
    // Set direction lines to output 
    rval = comedi_dio_config(*comedi_device, 
				config.dio_subdev, 
				config.dio_dir[i],
				COMEDI_OUTPUT);
    if (rval != 1) {
      snprintf(err_msg, ERR_SZ, "unable to configure dio_dir[%d]", i);
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,err_msg);
      ret_flag = FAIL;
    }
  } // End for i
 
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
int rt_cleanup(int level, void *comedi_device, RT_TASK *rt_task)
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
    if (comedi_close(comedi_device)!=0) {
      print_err_msg(__FILE__,__LINE__,__FUNCTION__,"unable to close comedi device");
      ret_flag = FAIL;
    }

  default:
    break;
  }

  fflush_printf("rt_cleanup complete\n");

  return ret_flag;
}


// ------------------------------------------------------------------
// Function: sigint_func
//
// ------------------------------------------------------------------
void sigint_func(int sig) {
  end = 1;
  return;
}



