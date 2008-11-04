#include <stdio.h>
#include <stdlib.h>
#include "yawff.h"
#include "util.h"
#include "test-util.h"

int main(int argc, char *argv[])
{
  array_t kine;
  config_t config;
  data_t data;
  int end_pos;
  int ret_val;

  // Initialize 
  init_test_config(&config);
  if (init_test_kine(&kine,config) != SUCCESS) {
    print_err_msg(
		  __FILE__,
		  __LINE__,
		  __FUNCTION__,
		  "unable to initialize kinematics"
		  );
  }
  if (init_test_data(&data,kine.nrow) != SUCCESS) {
    print_err_msg(
		  __FILE__,
		  __LINE__,
		  __FUNCTION__,
		  "unable to initialize data"
		  );
  }

  
  // Run yawff
  ret_val = yawff(kine,config,data,&end_pos);
  
  

  // Clean up
  free_test_kine(&kine);
  free_test_data(&data);

 
  return 0;
}



	    

