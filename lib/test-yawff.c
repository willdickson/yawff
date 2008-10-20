// ------------------------------------------------------------
// File: test-yawff.c
//
// Purpose: Set of unit tests for the yawff library
//
// Author: Will Dickson
//
// -------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <CUnit/Basic.h>
#include "yawff.h"
#include "test-util.h"

// Function prototypes -------------------------------------------
int testsuite_setup(void);
int testsuite_teardown(void);
void test_init_array(void);
void test_set_array_val(void);
void test_get_array_val(void);
void test_check_config(void);
void test_check_kine(void);
void test_check_compat(void);
void test_lowpass_filt1(void);
void test_integrator(void);

// --------------------------------------------------------------
// Function: main
//
// Purpose: sets up unit testing framework and runs the tests.
//
// --------------------------------------------------------------
int main(int argc, char *argv[]) 
{
  CU_pSuite pSuite = NULL;

  /* Initialize the CUnit test registry */
  if (CUE_SUCCESS != CU_initialize_registry()) {
    return CU_get_error();
  }
  
  // Add test suit
  pSuite = CU_add_suite("yawff testsuite", testsuite_setup, testsuite_teardown);
  if(pSuite == NULL) {
    print_err_msg(__FILE__,__LINE__, "adding testsuite failed");
    goto exit;
  }

  // Add tests - kind of a kludgey way to add tests
  if (NULL == CU_add_test(pSuite, "test of init_array()", test_init_array)) {
    print_err_msg(__FILE__,__LINE__,"adding test_init_array to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of set_array_val()", test_set_array_val)) {
    print_err_msg(__FILE__,__LINE__,"adding test_set_array_val to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of get_array_val()", test_get_array_val)) {
    print_err_msg(__FILE__,__LINE__,"adding test_get_array_val to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of check_config()", test_check_config)) {
    print_err_msg(__FILE__,__LINE__,"adding test_check_config to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of check_kine()", test_check_kine)) {
    print_err_msg(__FILE__,__LINE__,"adding test_check_kine to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of check_compat()", test_check_compat)) {
    print_err_msg(__FILE__,__LINE__,"adding test_check_compat to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of lowpass_filt1()", test_lowpass_filt1)) {
    print_err_msg(__FILE__,__LINE__,"adding test_lowpass_filt1 to testsuite failed");
    goto exit;
  }
  if (NULL == CU_add_test(pSuite, "test of integrator()", test_integrator)) {
    print_err_msg(__FILE__,__LINE__,"adding test_integrator to testsuite failed");
    goto exit;
  }
  
  // Run unit tests
  CU_basic_set_mode(CU_BRM_VERBOSE);
  freopen("/dev/null", "w", stderr); // Redirect stderr to get rid of annoying message
  CU_basic_run_tests();
  freopen("/dev/tty", "w", stderr);  // Return stderr to terminal

  // Clean up
 exit:
  CU_cleanup_registry();
  return CU_get_error();

}

// -------------------------------------------------------------
// Function: testsuit_setup
//
// Puropse: dummy testsuite setup function
//
// -------------------------------------------------------------
int testsuite_setup(void) {
  return 0;
}

// -------------------------------------------------------------
// Function: testsuite_teardown
// 
// Purpose: dummy testsuite teardown function
//
// -------------------------------------------------------------
int testsuite_teardown(void) {
  return 0;
}


// -------------------------------------------------------------
// Function: test_init_array
//
// Purpose: Unit test for init_array function
//
// -------------------------------------------------------------
void test_init_array(void)
{
  int nrow = 20;
  int ncol = 50;
  array_t array;
  
  // Int array creation
  CU_ASSERT(init_array(&array,nrow,ncol,INT_ARRAY)==SUCCESS);
  free_array(&array);

  // Float array creation
  CU_ASSERT(init_array(&array,nrow,ncol,FLT_ARRAY)==SUCCESS);
  free_array(&array);

  // Unkown array type creation - should fail
  CU_ASSERT(init_array(&array,nrow,ncol,UNKNOWN_ARRAY)==FAIL);
  
  // Bad nrow - should fail
  CU_ASSERT(init_array(&array,-1,ncol,UNKNOWN_ARRAY)==FAIL);
  
  // Bad ncol - should fail
  CU_ASSERT(init_array(&array,nrow,-1,UNKNOWN_ARRAY)==FAIL);
  
  return;
}

// -------------------------------------------------------------
// Function: test_set_array_val
//
// Purpose: Unit test for set_array_val function
//
// -------------------------------------------------------------
void test_set_array_val(void)
{
  int i,j;
  int s0,s1;
  int nrow = 15;
  int ncol = 10;
  array_t array;
  int ival;
  float fval;

  // Test integer arrays
  CU_ASSERT(init_array(&array,nrow,ncol,INT_ARRAY)==SUCCESS);
  for (i=0; i<nrow; i++) {
    for (j=0; j<ncol; j++) {
      ival = i + j;
      CU_ASSERT(set_array_val(&array,i,j,&ival)==SUCCESS);
    }
  }

  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol;j++) {
      ival = *((int*)( (array.data) + i*s0 + j*s1));
      CU_ASSERT(ival==i+j);
    } 
  }
  
  // Set values out of bounds - should FAIL 
  CU_ASSERT(set_array_val(&array,nrow,0,0)==FAIL);
  CU_ASSERT(set_array_val(&array,-1,0,0)==FAIL);
  CU_ASSERT(set_array_val(&array,0,ncol,0)==FAIL);
  CU_ASSERT(set_array_val(&array,0,-1,0)==FAIL);

  free_array(&array);

  // Test Float arrays
  CU_ASSERT(init_array(&array,nrow,ncol,FLT_ARRAY)==SUCCESS);
  for (i=0; i<nrow; i++) {
    for (j=0; j<ncol; j++) {
      fval = 10.0*(i + j);
      CU_ASSERT(set_array_val(&array,i,j,&fval)==SUCCESS);
    }
  }

  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol;j++) {
      fval = *((float*)( (array.data) + i*s0 + j*s1));
      CU_ASSERT(fval==10.0*(i+j));
    } 
  }
  free_array(&array);

  return;
}


// -------------------------------------------------------------
// Function: test_get_array_val
//
// Purpose: Unit test for get_array_val function
//
// -------------------------------------------------------------
void test_get_array_val(void)
{
  int i,j;
  int s0,s1;
  int nrow = 17;
  int ncol = 11;
  array_t array;
  int ival;
  float fval;

  // Test integer arrays
  init_array(&array,nrow,ncol,INT_ARRAY);
  for (i=0; i<nrow;i++) {
    for (j=0; j<ncol;j++) {
      ival = 2*i*j;
      CU_ASSERT(set_array_val(&array,i,j,&ival)==SUCCESS);
    }
  }
  
  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol; j++) {
      CU_ASSERT(get_array_val(&array,i,j,&ival)==SUCCESS);
      CU_ASSERT(ival == 2*i*j);
    }
  }

  // Get values out of bounds - should fail
  CU_ASSERT(get_array_val(&array,nrow,0,0)==FAIL);
  CU_ASSERT(get_array_val(&array,-1,0,0)==FAIL);
  CU_ASSERT(get_array_val(&array,0,ncol,0)==FAIL);
  CU_ASSERT(get_array_val(&array,0,-1,0)==FAIL);
  
  free_array(&array);

  // Test float arrays
  init_array(&array,nrow,ncol,FLT_ARRAY);
  for (i=0; i<nrow;i++) {
    for (j=0; j<ncol;j++) {
      fval = 0.5*i*j;
      CU_ASSERT(set_array_val(&array,i,j,&fval)==SUCCESS);
    }
  }
  
  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol; j++) {
      CU_ASSERT(get_array_val(&array,i,j,&fval)==SUCCESS);
      CU_ASSERT(fval == 0.5*i*j);
    }
  }
  free_array(&array);
}


// -------------------------------------------------------------
// Function: test_check_config 
//
// Purpose: Unit test for the check_config function.
//
// -------------------------------------------------------------
void test_check_config(void)
{
  config_t config;
  config_t config_test;

  init_test_config(&config);

  // Test for pass on good config
  CU_ASSERT(check_config(config)==SUCCESS);

  // Test for Fail on bad CLK/DIR 
  config_test = config;
  config_test.dio_clk[0] = config_test.dio_dir[1];
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Test for fail on bad kine map
  config_test = config;
  config_test.kine_map[0] = config_test.kine_map[2];
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Analog input range test
  config_test = config;
  config_test.yaw_ain = -1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  config_test = config;
  config_test.yaw_ain = MAX_AIN+1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  
  // Inertia range test
  config_test = config;
  config_test.yaw_inertia = 0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Yaw index to degree conversion test
  config_test = config;
  config_test.yaw_ind2deg = 0.5*FLT_EPSILON;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Torque limit range test
  config_test = config;
  config_test.yaw_torq_lim = 1.0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Yaw filter cutoff test
  config_test = config;
  config_test.yaw_filt_cut = -1.0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  
  // dt range tests
  config_test = config;
  config_test.dt = MAX_DT_NS + 1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  config_test.dt = MIN_DT_NS - 1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);  
}

// --------------------------------------------------------
// Function: test_check_kine
//
// Purpose: unit test for check_kine function
//
// --------------------------------------------------------
void test_check_kine(void)
{
  int i,j;
  int nrow = 100;
  int ncol = 5;
  array_t  kine;
  int val;

  init_array(&kine,nrow,ncol,INT_ARRAY);
  
  // Kinematics which should pass
  for (i=0; i<kine.nrow; i++) {
    for (j=0; j<kine.ncol; j++) {
      val = i + j;
      CU_ASSERT_FATAL(set_array_val(&kine,i,j,&val)==SUCCESS);
    }
  }
  CU_ASSERT(check_kine(kine)==SUCCESS);

  // Kinematics which should fail because too big step
  for (i=0; i<kine.nrow; i++) {
    for (j=0; j<kine.ncol; j++) {
      val = i*(j+1);
      CU_ASSERT_FATAL(set_array_val(&kine,i,j,&val)==SUCCESS);
    }
  }
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  
  // Set to ok kinematics
  for (i=0; i<kine.nrow; i++) {
    for (j=0; j<kine.ncol; j++) {
      val = i + j;
      set_array_val(&kine,i,j,&val);
    }
  }
  // Kinematics which should fail with bad # of rows
  kine.nrow = 0;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Kinematics which should fail with bad # columns
  kine.nrow = nrow;
  kine.ncol = -3;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Kinematics which should fail with bad stride s0
  kine.ncol = ncol;
  kine.s0 = 0;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Kinematics which should fail with bad stride s1
  kine.s0 = ncol*sizeof(int);
  kine.s1 = 0;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Should fail with unkown type
  kine.s1 = sizeof(int);
  kine.type = UNKNOWN_ARRAY;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);

  // Clean up
  free_array(&kine);
  return;
}


// -----------------------------------------------------------
// Function: test_check_compat
//
// Purpose: unit test for check_compat function
//
// ------------------------------------------------------------
void test_check_compat(void)
{
  array_t kine;
  config_t config;

  // Setup config structure
  init_test_config(&config);

  // Setup kinematics
  init_test_kine(&kine, config);
    
  // Kinematics and config which should be compatible
  CU_ASSERT(check_compat(config,kine)==SUCCESS);

  // Kinematics and config which shouldn't be compatible
  config.num_motor = KINE_NCOL+2;
  CU_ASSERT_FALSE(check_compat(config,kine)==SUCCESS);

  free_array(&kine);

  return;
}


// -------------------------------------------------------------
// Function: test_lowpass_filt1
//
// Purpose: Unit test for lowpass_filt1 function. Currently only 
// tests the gain of the filter against the theoretical value. 
//
// --------------------------------------------------------------
void test_lowpass_filt1(void)
{
  float f_cut;
  float t;
  float x;
  float y;
  float x_max;
  float y_max ;
  float gain_meas;
  float gain_calc;
  float gain_err ;
  float a,b;
  int i,j;

  // Loop over test values
  for (j=0; j<NUM_TESTVALS; j++) {

    // Check sinewave with freq << freq_cut
    t = 0;
    y = 0;
    a = (F_TEST_MAX - F_TEST_MIN)/((float) NUM_TESTVALS);
    b = F_TEST_MIN;
    f_cut = a*((float) j) + b;
    x_max = 0;
    y_max = 0;


    for (i=0; i< (int)(NUM_CYCLE/(F_SINE*FILT_DT)); i++) {
      x = sinf(2*M_PI*t*F_SINE);
      y = lowpass_filt1(x,y,f_cut,FILT_DT);
      t+=FILT_DT;
      // Get max values of x and y for amplitude estimate
      if ((i>(int)(TEST_CYCLE_MIN/(F_SINE*FILT_DT))) && 
	  (i< (int)(TEST_CYCLE_MAX/(F_SINE*FILT_DT)))) {
	x_max = x > x_max ? x : x_max;
	y_max = y > y_max ? y : y_max;
      }
    }
    
    // Compute the gain error
    gain_meas = y_max/x_max;
    gain_calc = 1.0/sqrtf(1.0 + pow(F_SINE/f_cut,2));
    gain_err = fabs(gain_meas - gain_calc)/gain_calc;    
    CU_ASSERT(gain_err < FILT_GAIN_TOL);
  };

  return;
}

// ------------------------------------------------------------
// Function: test_integrator
//
// Purpose: Unit test for integrator function. Note, this is
// test is kind of cheesey as it simple integrates a simple
// ballistic trajectory and 
//
// ------------------------------------------------------------
void test_integrator(void)
{
  int N = (int)(INTEG_T/INTEG_DT);
  state_t state_kutta[N];
  state_t state_euler[N];
  float pos;
  float vel;
  float err_kutta;
  float err_euler;
  float pos_max;
  float pos_min;
  float t ;
  int i;
  int ret_val;
  int test_flag;

  // Initial conditions
  state_kutta[0].pos = POS_INIT;
  state_kutta[0].vel = VEL_INIT;
  state_euler[0].pos = POS_INIT;
  state_euler[0].vel = VEL_INIT;

  // Integrate test case
  test_flag = SUCCESS;
  for (i=0; i<N-1; i++) {

    // Runge-Kutta Integration
    ret_val = integrator(
			 state_kutta[i],
			 &state_kutta[i+1],
			 FORCE,
			 MASS,
			 DAMPING,
			 INTEG_DT,
			 INTEG_RKUTTA
			 );
    if (ret_val == FAIL) {
      test_flag = FAIL;
    }

    // Euler Integration
    ret_val = integrator(
			 state_euler[i],
			 &state_euler[i+1],
			 FORCE,
			 MASS,
			 DAMPING,
			 INTEG_DT,
			 INTEG_EULER
			 );
    if (ret_val == FAIL) {
      test_flag = FAIL;
    }
  }
  CU_ASSERT(test_flag==SUCCESS);

  // Check test case against exact solution
  t = 0.0;
  err_kutta = 0.0;
  err_euler = 0.0;
  pos_max = POS_INIT;
  pos_min = POS_INIT;
  for (i=1; i<N; i++) {
    t += INTEG_DT;
    // Exact solution for pos and vel
    pos = POS_INIT + VEL_INIT*t + 0.5*(FORCE/MASS)*pow(t,2);
    vel = VEL_INIT + (FORCE/MASS)*t;
    
    // Compute absolute cummulative err 
    err_kutta += fabs(state_kutta[i].pos - pos);
    err_euler += fabs(state_euler[i].pos - pos);

    // Get max and min positions for total displacement
    pos_max = pos_max>pos? pos_max : pos;
    pos_min = pos_min<pos? pos_min : pos;    
  }
  
  // Compute mean cummulative error divided by total displacement
  err_kutta = err_kutta/(float)(N-1);
  err_euler = err_euler/(float)(N-1);
  err_kutta = err_kutta/(pos_max - pos_min);
  err_euler = err_kutta/(pos_max - pos_min);

  CU_ASSERT(err_kutta < INTEG_TOL);
  CU_ASSERT(err_euler < INTEG_TOL);

  // Check for failure when given unknown integrator number
  ret_val = integrator(
		       state_euler[0],
		       &state_euler[1],
		       FORCE,
		       MASS,
		       DAMPING,
		       INTEG_DT,
		       INTEG_UNKNOWN
		       );
  CU_ASSERT_FALSE(ret_val == SUCCESS);  
  return;
}



