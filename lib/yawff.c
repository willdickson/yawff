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

// Global variables
int sigint_flag = 0;

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
  
  // Startup method 
  printf("\n");
  printf("                  Starting yawff \n");
  printf("=======================================================\n");

  // Check configuration
  print_config(config);
  if (check_config(config)==FAIL) {
    print_err_msg(__FILE__,__LINE__,"bad configuration");
    return FAIL;
  }

  // Check kinematics
  if (check_kine(kine)==FAIL) {
    print_err_msg(__FILE__,__LINE__,"kinematics invalid");
    return FAIL;
  }

  // Check that compatibility
  if (check_compat(config,kine)==FAIL){
    print_err_msg(__FILE__,__LINE__,"compatibility");
    return FAIL;
  }

  // Initialize RTAI

  // Initialize comedi device

  // Take zero reading from yaw torque sensor

 
  
  
  // Temporary
  return SUCCESS;
}


// ------------------------------------------------------------------
// Function: sigint_func
//
// ------------------------------------------------------------------
void sigint_func(int sig) {
  sigint_flag = 1;
  return;
}



