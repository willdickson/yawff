// -------------------------------------------------------------------
// File: yawff.c
//
// Purpose: 
// 
// Author: Will Dickson 09/30/2008
// -------------------------------------------------------------------
#include "yawff.h"

// RT-task parameters
const int PRIORITY=1;
const int STACK_SIZE=4096;
const int MSG_SIZE=0;


// Function prototypes
int init_comedi(void);
int init_rtai(void);

// Global variables
volatile int sigint_flag = 0;

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


  void *sighandler = NULL;
  int rtn_flag = SUCCESS;

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

  // Initialize RTAI

  // Initialize comedi device

  // Take zero reading from yaw torque sensor


 
  // Clean up


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
// Function: sigint_func
//
// ------------------------------------------------------------------
void sigint_func(int sig) {
  sigint_flag = 1;
  return;
}



