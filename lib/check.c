// -----------------------------------------------------------------
// File: check.c
//
// Purpose: Contains functions used for checking that configurtation
// and kine types are valid.
//
// Functions:
//
//   check_config    = checks if configuration structure is valid
//   check_ranges    = checks if ranges in configuration structure
//                     are valid
//   check_clkdir    = checks if the  clk/dir to dio pin assignment
//                     is valid/ 
//   check_kine_map  = checks if the map from kinematics to motors
//                     is valid
//
// Author: Will Dickson 
//
// -----------------------------------------------------------------
#include "yawff.h"

// ----------------------------------------------------------------
// Function: check_config
//
// Purpose: Checks that configuration is valid
//
// ----------------------------------------------------------------
int check_config(config_t config)
{
  if (check_ranges(config) == FAIL) {
    return FAIL;
  }
  if (check_clkdir(config) == FAIL) {
    return FAIL;
  }
  if (check_kine_map(config) == FAIL) {
    return FAIL;
  }
  return SUCCESS;
}

// ----------------------------------------------------------------
// Function: check ranges
//
// Purpose: checks that configuration values are in range
// 
// ----------------------------------------------------------------
int check_ranges(config_t config)
{
  int flag = SUCCESS;

  // Check number of motors
  if ((config.num_motor <= 0) || (config.num_motor>MAX_MOTOR)) {
    print_err_msg(__FILE__,__LINE__,"incorrect number of motors");
    flag = FAIL;
  }

  // Check yaw motor range
  if ((config.yaw_motor < 0) || (config.yaw_motor >= config.num_motor)) {
    print_err_msg(__FILE__,__LINE__,"yaw motor out of range");
    flag = FAIL;
  }
  // Check analog input range
  if ((config.yaw_ain < 0) || (config.yaw_ain > MAX_AIN)) {
    print_err_msg(__FILE__,__LINE__,"yaw_ain out of range");
    flag = FAIL;
  }

  // Check voltage to torque calibration
  if (config.yaw_volt2torq <=0) {
    print_err_msg(__FILE__,__LINE__,"yaw_volt2torq <= 0");
    flag = FAIL;
  }

  // Check yaw inertia range
  if (config.yaw_inertia <= 0 ) {
    print_err_msg(__FILE__,__LINE__,"yaw_inertia <= 0");
    flag = FAIL;
  }

  // Check yaw index to degree conversion range
  if (fabs(config.yaw_ind2deg) < FLT_EPSILON) {
    print_err_msg(__FILE__,__LINE__,"yaw_inertia < FLT_EPSILON");
    flag = FAIL;
  }

  // Check torque limit range
  if ((config.yaw_torq_lim < MIN_TORQ_LIM) || (config.yaw_torq_lim > MAX_TORQ_LIM)) {
    print_err_msg(__FILE__,__LINE__,"yaw_torq_lim out of range");
    flag = FAIL;
  }

  // Check yaw filter cutoff frequency
  if (config.yaw_filt_cut < 0.0) {
    print_err_msg(__FILE__,__LINE__,"yaw_filt_cut < 0");
    flag = FAIL;
  }

  // Check realtime step range
  if ((config.dt > MAX_DT_NS) || (config.dt < MIN_DT_NS)) {
    print_err_msg(__FILE__,__LINE__,"dt out of range");
    flag = FAIL;
  }

  return flag;
}

// ----------------------------------------------------------------
// Function: check_clkdir
//
// Purpose: checks that clk and dir configuration is valid.
//
// ----------------------------------------------------------------
int check_clkdir(config_t config)
{
  int i,j;
  char err_msg[ERR_SZ];
  int flag = SUCCESS;

  // Check clk and dir configuration
  for (i=0; i<config.num_motor; i++) {   
    // Check range
    if ((config.dio_clk[i] < 0) || (config.dio_clk[i] > MAX_DIO)) {
      snprintf(err_msg, ERR_SZ, "clk[%d] out of range", i);
      print_err_msg(__FILE__,__LINE__,err_msg);
      flag = FAIL;
    }
    if ((config.dio_dir[i] < 0) || (config.dio_dir[i] > MAX_DIO)) {
      snprintf(err_msg, ERR_SZ, "dir[%d] out of range", i);
      print_err_msg(__FILE__,__LINE__,err_msg);
      flag = FAIL;
    }
    // Check uniqness
    if (config.dio_clk[i] == config.dio_dir[i]) {
      snprintf(err_msg, ERR_SZ, "clk/dir dio not unique, clk[%d] = dir[%d]", i,i);
      print_err_msg(__FILE__,__LINE__,err_msg);
      flag = FAIL;
    }
    if (i<config.num_motor) {
      for (j=i+1; j<config.num_motor; j++) {
	if (config.dio_clk[i] == config.dio_clk[j]) {
	  snprintf(err_msg, ERR_SZ, "clk/dir dio not unique, clk[%d] = clk[%d]", i,j);
	  print_err_msg(__FILE__,__LINE__,err_msg);
	  flag = FAIL;
	}
	if (config.dio_clk[i] == config.dio_dir[j]) {
	  snprintf(err_msg, ERR_SZ, "clk/dir dio not unique, clk[%d] = dir[%d]", i,j);
	  print_err_msg(__FILE__,__LINE__,err_msg);
	  flag = FAIL;
	}
	if (config.dio_dir[i] == config.dio_clk[j]) {
	  snprintf(err_msg, ERR_SZ, "clk/dir dio not unique, dir[%d] = clk[%d]", i,j);
	  print_err_msg(__FILE__,__LINE__,err_msg);
	  flag = FAIL;
	}
	if (config.dio_dir[i] == config.dio_dir[j]) {
	  snprintf(err_msg, ERR_SZ, "clk/dir dio not unique, dir[%d] = dir[%d]", i,j);
	  print_err_msg(__FILE__,__LINE__,err_msg);
	  flag = FAIL;
	}
      } 
    } 
  } 
  return flag;
}

// ----------------------------------------------------------------
// Function: check_kine_map
//
// Purpose: check that kinematics map is valid
//
// ----------------------------------------------------------------
int check_kine_map(config_t config)
{
  int i,j;
  char err_msg[ERR_SZ];
  int flag = SUCCESS;
  
    // Check kinematics to motor map
  for (i=0; i< (config.num_motor-1); i++) {    
    // Check range
    if ((config.kine_map[i] < 0) || (config.kine_map[i] > (MAX_MOTOR-1))) {
      snprintf(err_msg, ERR_SZ, "kine_map[%d] out of range",i);
      print_err_msg(__FILE__,__LINE__,err_msg);
      return FAIL;
    }
    // check against yaw motor number
    if (config.kine_map[i] == config.yaw_motor) {
      snprintf(err_msg, ERR_SZ, "kinematicss map overlap w/ yaw motor, kine_map[%d] = yaw_motor",i);
      print_err_msg(__FILE__,__LINE__,err_msg);
      flag = FAIL;
    }
    // Check uniqness w/ self
    if (i < (config.num_motor-2)) {
      for (j=i+1; j<(config.num_motor-1); j++) {
	if (config.kine_map[i] == config.kine_map[j]) {
	  snprintf(err_msg, ERR_SZ, "kinematicss map not unique, kine_map[%d] = kine_map[%d]",i,j);
	  print_err_msg(__FILE__,__LINE__,err_msg);
	  flag = FAIL;
	}
      } 
    } 
  } 
  return flag;
}

// ----------------------------------------------------------------
// Function: check_kine
//
// Purpose: Check that kinematics are valid
//
// ----------------------------------------------------------------
int check_kine(array_t kine)
{
  int i,j;
  int flag = SUCCESS;
  int p0,p1;
  int err_flag;

  if (kine.nrow <= 0) {
    print_err_msg(__FILE__,__LINE__,"number of rows <= 0");
    return  FAIL;
  }
  if (kine.ncol <= 0) {
    print_err_msg(__FILE__,__LINE__,"number of columns  <= 0");
    return FAIL;
  }
  if (kine.s0 ==0){
    print_err_msg(__FILE__,__LINE__,"stride s0 == 0");
    return FAIL;
  }
  if (kine.s1 ==0){
    print_err_msg(__FILE__,__LINE__,"stride s1 == 0");
    return FAIL;
  }
  for (i=0;i<(kine.nrow-1); i++) {
    for (j=0; j<kine.ncol; j++) {
      //err_flag = get_kine_pos(&kine,j,i, &p0);
      //err_flag = get_kine_pos(&kine,j,i+1, &p1);
      err_flag = get_array_val(&kine,i,j,&p0);
      err_flag = get_array_val(&kine,i+1,j,&p1);
      if (abs(p1-p0) > 1) {
	flag = FAIL;
      }
    }
  }
  if (flag==FAIL) {
    print_err_msg(__FILE__,__LINE__,"kinematics contain steps > 1");
  }
  return flag;
}

// ---------------------------------------------------------------
// Function: check_compat
//
// Purpose: checks that the kinematic and the configuration are
// compatible.
//
// ----------------------------------------------------------------
int check_compat(config_t config, array_t kine)
{
  int flag = SUCCESS;
  if (kine.ncol != config.num_motor) {
    print_err_msg(__FILE__, __LINE__, "kinematics and configuration are incompatible");
    flag = FAIL;
  }
  return flag;
}
