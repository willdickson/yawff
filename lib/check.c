/*---------------------------------------------------------------------
  yawff
  Copyright (C) William Dickson, 2008.
  
  wbd@caltech.edu
  www.willdickson.com

  Released under the LGPL Licence, Version 3
  
  This file is part of yawff.

  yawff is free software: you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.
    
  yawff is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General
  Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with yawff.  If not, see
  <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------
  check.c

  Purpose: Contains functions used for checking that the system 
  configurtation structure, the kinemeatic array and structure of
  output data arrays types are valid and compatible.

  Functions:

   check_yawff_input = check inputs to yawff function
   check_yawff_w_ctlr_input = check inputs to yawff_w_ctlr function
   check_config       = checks if configuration structure is valid
   check_ranges       = checks if ranges in configuration structure
                        are valid
   check_clkdir       = checks if the  clk/dir to dio pin assignment
                        is valid/ 
   check_kine_map     = checks if the map from kinematics to motors
                        is valid
   check_kine         = checks that kinematics array is valid
   check_kine_compat  = checks that kinematics array and 
                        configuration structure are compatible
   check_array        = check that an array is valid
   check_data         = check that structure of data arrays is 
                        valid
   check_data_compat  = check that structur of data arrays and 
                        kinematics array are compatible
   check_setpt_compat = check that kinematics and setpt arrays are 
                        compatible
   check_u_compat     = check that kinematics and u arrays are compatible
   check_motor_cal    = check that motor calibration satisfies the
                        correct assumptions
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#include "yawff.h"
#include "check.h"
#include "util.h"


// ------------------------------------------------------------------
// Function: check_yawff_input
//
// Purpose: Checks the inputs to the yawff function.
//
// Arguments:
//   kine    = array of wing kinematics 
//   config  = system configuration structure
//   data    = structure of return data arrays
//
// Return: SUCCESS or FAIL
//
// ------------------------------------------------------------------
int check_yawff_input(array_t kine, config_t config, data_t data)
{
  int rtn_flag = SUCCESS;

  // Check configuration
  if (check_config(config) != SUCCESS) {
    PRINT_ERR_MSG("bad configuration");
    rtn_flag = FAIL;
  }

  // Check kinematics
  if (check_kine(kine) != SUCCESS) {
    PRINT_ERR_MSG("kinematics invalid");
    rtn_flag = FAIL;
  }

  // Check that kinematics configuration compatibility
  if (check_kine_compat(config,kine) != SUCCESS){
    PRINT_ERR_MSG("kinematics incompatible");
    rtn_flag = FAIL;
  }
  
  // Check data
  if (check_data(data) != SUCCESS) {
    PRINT_ERR_MSG("data invalid");
    rtn_flag = FAIL;
  }

  // Check data compatibility
  if (check_data_compat(kine,data)!=SUCCESS) {
    PRINT_ERR_MSG("data incompatible");
    rtn_flag = FAIL;
  }

  return rtn_flag;
}


// ------------------------------------------------------------------
// Function: check_yawff_w_ctlr_input
//
// Purpose: Checks the inputs to the yawff function.
//
// Arguments:
//   kine    = array of wing kinematics 
//   config  = system configuration structure
//   data    = structure of return data arrays
//
// Return: SUCCESS or FAIL
//
// ------------------------------------------------------------------
int check_yawff_w_ctlr_input(
        array_t setpt, 
        config_t config, 
        array_t kine, 
        array_t u, 
        data_t data)
{
  int rtn_flag = SUCCESS;

  // Check configuration
  if (check_config(config) != SUCCESS) {
    PRINT_ERR_MSG("bad configuration");
    rtn_flag = FAIL;
  }

  // Check setpt array
  if (check_array(setpt) != SUCCESS) {
    PRINT_ERR_MSG("setpt array invalid");
    rtn_flag = FAIL;
  }

  // Check kinematics - make sure is valid array
  if (check_array(kine) != SUCCESS) {
    PRINT_ERR_MSG("kinematics array invalid");
    rtn_flag = FAIL;
  }

  // Check that kinematics configuration compatibility
  if (check_kine_compat(config,kine) != SUCCESS){
    PRINT_ERR_MSG("kinematics incompatible");
    rtn_flag = FAIL;
  }
  
  // Check data
  if (check_data(data) != SUCCESS) {
    PRINT_ERR_MSG("data invalid");
    rtn_flag = FAIL;
  }

  // Check data compatibility
  if (check_data_compat(kine,data)!=SUCCESS) {
    PRINT_ERR_MSG("data and kinematics incompatible");
    rtn_flag = FAIL;
  }

  // Check setpt compatibility
  if (check_setpt_compat(kine,setpt)!=SUCCESS) {
    PRINT_ERR_MSG("setpt and kinematics incompatible");
    rtn_flag = FAIL;
  }

  // Check u compatibility
  if (check_u_compat(kine,u)!=SUCCESS) {
      PRINT_ERR_MSG("u and kinematics incompatible");
      rtn_flag = FAIL;
  }

  return rtn_flag;
}

// ----------------------------------------------------------------
// Function: check_config
//
// Purpose: Checks that system configuration is valid.
//
// Argument:
//   config = system configuration structure.
//
// Return: SUCCESS or FAIL
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
// Purpose: Checks that configuration values within acceptable
// ranges.
//
// Arguments:
//   config = system configuration structure.
//
// Return: SUCCESS or FAIL
// 
// ----------------------------------------------------------------
int check_ranges(config_t config)
{
  int i;
  int test;
  int flag = SUCCESS;
  char err_msg[ERR_SZ];

  // Check number of motors
  if ((config.num_motor <= 0) || (config.num_motor>MAX_MOTOR)) {
    PRINT_ERR_MSG("incorrect number of motors");
    flag = FAIL;
  }

  // Check yaw motor range
  if ((config.yaw_motor < 0) || (config.yaw_motor >= config.num_motor)) {
    PRINT_ERR_MSG("yaw motor out of range");
    flag = FAIL;
  }

  // Check dio_disbale range
  if ((config.dio_disable < 0) || (config.dio_disable > MAX_DIO)) {
    PRINT_ERR_MSG("dio_disable out of range");
    flag = FAIL;
  } 
  
  // Check analog input range
  if (config.yaw_ain > MAX_AIN) {
    PRINT_ERR_MSG("yaw_ain out of range");
    flag = FAIL;
  }

  // Check zeroing sample interval
  if (config.yaw_ain_zero_dt < AIN_ZERO_DT_MIN) {
      PRINT_ERR_MSG("yaw_ain_zero_dt < AIN_ZERO_DT_MIN");
      flag = FAIL;
  }

  // Check that number of zeroing samples is greater than zero
  if (config.yaw_ain_zero_num == 0) {
      PRINT_ERR_MSG("yaw_ain_zero_dt == 0");
      flag = FAIL;
  }

  // Check yaw inertia range
  if (config.yaw_inertia <= 0 ) {
    PRINT_ERR_MSG("yaw_inertia <= 0");
    flag = FAIL;
  }

  // Check yaw index to degree conversion range
  if (fabs(config.yaw_ind2deg) < FLT_EPSILON) {
    PRINT_ERR_MSG("yaw_inertia < FLT_EPSILON");
    flag = FAIL;
  }

  // Check torque limit range
  if ((config.yaw_torq_lim < MIN_TORQ_LIM) || (config.yaw_torq_lim > MAX_TORQ_LIM)) {
    PRINT_ERR_MSG("yaw_torq_lim out of range");
    flag = FAIL;
  }

  // Check yaw torque deadband
  if ((config.yaw_torq_deadband < 0)) {
      PRINT_ERR_MSG("yaw_torq_deadband must be >= 0");
      flag = FAIL;
  }

  // Check yaw lowpass filter cutoff frequency
  if (config.yaw_filt_lpcut < 0.0) {
    PRINT_ERR_MSG("yaw_filt_lpcut < 0");
    flag = FAIL;
  }

  // Check yaw highpss filter cutoff frequency
  if (config.yaw_filt_hpcut < 0.0) {
      PRINT_ERR_MSG("yaw_filt_hpcut < 0");
      flag = FAIL;
  }

  // Check damping constant
  if (config.yaw_damping  < 0.0) {
    PRINT_ERR_MSG("damping < 0");
    flag = FAIL;
  }

  // Check realtime step range
  if ((config.dt > MAX_DT_NS) || (config.dt < MIN_DT_NS)) {
    PRINT_ERR_MSG("dt out of range");
    flag = FAIL;
  }

  // Check integrator
  if ((config.integ_type != INTEG_EULER) && (config.integ_type != INTEG_RKUTTA)) {
      PRINT_ERR_MSG("unknown integrator");
      flag = FAIL;
  }

  // Check ff_flag
  if ((config.ff_flag != FF_ON) && (config.ff_flag != FF_OFF)) {
      PRINT_ERR_MSG("unknown ff_flag value");
      flag = FAIL;
  }

  // Check ctlr_flag
  test = FALSE;
  if (config.ctlr_flag == CTLR_ON) {
    test = TRUE;
  }
  if (config.ctlr_flag == CTLR_OFF){
    test = TRUE;
  }
  if (test == FALSE) {
    PRINT_ERR_MSG("unknown ctlr_flag value");
    flag = FAIL;
  }

  // Tests specific to case where ctlr_flag is true
  if (config.ctlr_flag == CTLR_ON) {

    // Check the controller type
    test = FALSE;
    if (config.ctlr_param.type == CTLR_TYPE_VEL) {
      test = TRUE;
    }
    if (config.ctlr_param.type == CTLR_TYPE_POS) {
      test = TRUE;
    }
    if (test == FALSE) {
      PRINT_ERR_MSG("unknown controller type");
      flag = FAIL;
    }

    // Check motor calibrations
    for (i=0; i<config.num_motor; i++) {
      if (check_motor_cal(config.motor_cal[i]) != SUCCESS) {
        snprintf(err_msg,ERR_SZ,"motor[%d] calibration invalid",i);
        PRINT_ERR_MSG(err_msg);
        flag = FAIL;
      }
    }

  } // end if (config.ctlr_flag) 

  return flag;
}

// ----------------------------------------------------------------
// Function: check_clkdir
//
// Purpose: checks that clk and dir configuration in the system 
// configuration is valid.
//
// Arguments:
//   config = system configuratio structure.
//
// Return: SUCCESS or FAIL
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
      PRINT_ERR_MSG(err_msg);
      flag = FAIL;
    }
    if ((config.dio_dir[i] < 0) || (config.dio_dir[i] > MAX_DIO)) {
      snprintf(err_msg, ERR_SZ, "dir[%d] out of range", i);
      PRINT_ERR_MSG(err_msg);
      flag = FAIL;
    }
    // Check collision with dio_disable
    if (config.dio_dir[i] == config.dio_disable) {
      snprintf(err_msg, ERR_SZ, "dir[%d] equal dio_disable", i);
      PRINT_ERR_MSG(err_msg);
      flag = FAIL;
    }
    if (config.dio_clk[i] == config.dio_disable) {
      snprintf(err_msg, ERR_SZ, "dir[%d] equal dio disable", i);
      PRINT_ERR_MSG(err_msg);
      flag = FAIL;
    }

    // Check uniqness
    if (config.dio_clk[i] == config.dio_dir[i]) {
      snprintf(err_msg, ERR_SZ, "clk/dir dio, clk[%d] = dir[%d]", i,i);
      PRINT_ERR_MSG(err_msg);
      flag = FAIL;
    }
    if (i<config.num_motor) {
      for (j=i+1; j<config.num_motor; j++) {
        if (config.dio_clk[i] == config.dio_clk[j]) {
	        snprintf(err_msg, ERR_SZ, "clk/dir dio not unique, clk[%d] = clk[%d]", i,j);
	        PRINT_ERR_MSG(err_msg);
	        flag = FAIL;
	      }
	      if (config.dio_clk[i] == config.dio_dir[j]) {
	        snprintf(err_msg, ERR_SZ, "clk/dir dio clk[%d] = dir[%d]", i,j);
	        PRINT_ERR_MSG(err_msg);
	        flag = FAIL;
	      }
	      if (config.dio_dir[i] == config.dio_clk[j]) {
	        snprintf(err_msg, ERR_SZ, "clk/dir dio, dir[%d] = clk[%d]", i,j);
	        PRINT_ERR_MSG(err_msg);
	        flag = FAIL;
	      }
	      if (config.dio_dir[i] == config.dio_dir[j]) {
	        snprintf(err_msg, ERR_SZ, "clk/dir dio, dir[%d] = dir[%d]", i,j);
	        PRINT_ERR_MSG(err_msg);
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
// Purpose: check that kinematics the mapping from the wing 
// kinematics array (columns) to the DIO given in the system
// configuration structure is valid.
//
// Argument:
//   config = system configuration structure.
//
// Return: SUCCESS or FAIL
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
      snprintf(err_msg, ERR_SZ,"kine_map[%d] out of range",i);
      PRINT_ERR_MSG(err_msg);
      return FAIL;
    }
    // check against yaw motor number
    if (config.kine_map[i] == config.yaw_motor) {
      snprintf(err_msg, ERR_SZ,"kinematicss map overlap w/ yaw motor, kine_map[%d] = yaw_motor",i);
      PRINT_ERR_MSG(err_msg);
      flag = FAIL;
    }
    // Check uniqness w/ self
    if (i < (config.num_motor-2)) {
      for (j=i+1; j<(config.num_motor-1); j++) {
	      if (config.kine_map[i] == config.kine_map[j]) {
	       snprintf(err_msg, ERR_SZ, "kinematicss map not unique, kine_map[%d] = kine_map[%d]",i,j);
	       PRINT_ERR_MSG(err_msg);
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
// Purpose: Check that the kinematics array is valid.
//
// Argument:
//   kine = kinematics array.
//
// Return: SUCCESS or FAIL
//
// ----------------------------------------------------------------
int check_kine(array_t kine)
{
  int i,j;
  int flag = SUCCESS;
  int p0,p1;
  int err_flag;

  if (check_array(kine)==FAIL) {
    PRINT_ERR_MSG("invalid array");
    return FAIL;
  }

  for (i=0;i<(kine.nrow-1); i++) {
    for (j=0; j<kine.ncol; j++) {
      err_flag = get_array_val(kine,i,j,&p0);
      err_flag = get_array_val(kine,i+1,j,&p1);
      if (abs(p1-p0) > 1) {
	flag = FAIL;
      }
    }
  }
  if (flag==FAIL) {
    PRINT_ERR_MSG("kinematics contain steps > 1");
  }
  return flag;
}

// ---------------------------------------------------------------
// Function: check_compat
//
// Purpose: checks that the kinematic array and the system 
// configuration are compatible.
//
// Arguments:
//   config = system configuration structure
//   kine   = kinematics array
//
// Return: SUCCESS or FAIL
//
// ----------------------------------------------------------------
int check_kine_compat(config_t config, array_t kine)
{
  int flag = SUCCESS;
  if (kine.ncol != config.num_motor) {
    PRINT_ERR_MSG("kinematics and configuration are incompatible");
    flag = FAIL;
  }
  return flag;
}


// -----------------------------------------------------------------
// Function: check_array
//
// Purpose: checks that an array is valid.
//
// Arguments:
//   array =  an array structure.
//
// Return: SUCCESS or FAIL
//
// -----------------------------------------------------------------
int check_array(array_t array)
{
  int flag = SUCCESS;
  if (array.nrow <= 0) {
    PRINT_ERR_MSG("number of rows <= 0");
    flag = FAIL;
  }
  if (array.ncol <= 0) {
    PRINT_ERR_MSG("number of columns  <= 0");
    flag = FAIL;
  }
  if (array.s0 ==0){
    PRINT_ERR_MSG("stride s0 == 0");
    flag = FAIL;
  }
  if (array.s1 ==0){
    PRINT_ERR_MSG("stride s1 == 0");
    flag = FAIL;
  }
  return flag;
}

// -----------------------------------------------------------------
// Function: check_data
//
// Purpose: checks that the structure of output data array's is 
// valid.
//
// Arguments:
//   data = structure of output arrays
// 
// Return: SUCCESS or FAIL
//
// -----------------------------------------------------------------
int check_data(data_t data)
{
  int flag=SUCCESS;

  // Check that arrays are valid
  if (check_array(data.t)==FAIL) {
    PRINT_ERR_MSG("t array invalid");
    flag = FAIL;
  }
  if (check_array(data.pos)==FAIL) {
    PRINT_ERR_MSG("pos array invalid");
    flag = FAIL;
  }
  if (check_array(data.vel)==FAIL) {
    PRINT_ERR_MSG("vel array invalid");
    flag = FAIL;
  }
  if (check_array(data.torq)==FAIL) {
    PRINT_ERR_MSG("torq array invalid");
    flag = FAIL;
  }

  // Check that t, pos, and vel arrays are Nx1
  if (data.t.ncol != 1) {
    PRINT_ERR_MSG("t array not Nx1");
    flag = FAIL;
  }
  if (data.pos.ncol != 1) {
    PRINT_ERR_MSG("pos array not Nx1");
    flag = FAIL;
  }
  if (data.vel.ncol != 1) {
    PRINT_ERR_MSG("vel array not Nx1");
    flag = FAIL;
  }
  
  // Check that torq array is  Nx2
  if (data.torq.ncol != 2) {
    PRINT_ERR_MSG("torq array not Nx1");
    flag = FAIL;
  }

  // Check that time is double array 
  if (data.t.type != DBL_ARRAY) {
    PRINT_ERR_MSG("t array not DBL_ARRAY");
    flag = FAIL;
  }

  // Check that all other arrays are float arrays
  if (data.pos.type != FLT_ARRAY) {
    PRINT_ERR_MSG("pos array not FLT_ARRAY");
    flag = FAIL;
  }
  if (data.vel.type != FLT_ARRAY) {
    PRINT_ERR_MSG("vel array not FLT_ARRAY");
    flag = FAIL;
  }
  if (data.torq.type != FLT_ARRAY) {
    PRINT_ERR_MSG("torq array not FLT_ARRAY");
    flag = FAIL;
  }
  return flag;
}

// -------------------------------------------------------------
// Function: check_data_compat
//
// Purpose: check that structure of data arrays and kinematics 
// array are compatible.
//
// Arguments:
//   kine  = kinematics array
//   data  = structure of data arrays
//
// Return: SUCCESS or FAIL
//
// -------------------------------------------------------------
int check_data_compat(array_t kine, data_t data)
{
  int flag = SUCCESS;
  
  if ((kine.nrow != data.t.nrow)) {
    PRINT_ERR_MSG("t array incompatible with kinematics");
    flag = FAIL;
  }
  if ((kine.nrow != data.pos.nrow)) {
    PRINT_ERR_MSG("pos array incompatible with kinematics");
    flag = FAIL;

  }
  if ((kine.nrow != data.vel.nrow)) {
    PRINT_ERR_MSG("vel array incompatible with kinematics");
    flag = FAIL;
  }
  if ((kine.nrow != data.torq.nrow)) {
    PRINT_ERR_MSG("torq array incompatible with kinematics");
    flag = FAIL;
  }
  return flag;
}

// -------------------------------------------------------------
// Function: check_setpt_compat
//
// Purpose: check that kinematics and setpt arrays are compatible.
//
// Arguments:
//   kine   = kinematics array
//   setpt  = setpt array 
//
// Return: SUCCESS or FAIL
//
// -------------------------------------------------------------
int check_setpt_compat(array_t kine, array_t setpt)
{
  int flag = SUCCESS;
  
  if ((kine.nrow != setpt.nrow)) {
    PRINT_ERR_MSG("setpt array incompatible with kinematics array");
    flag = FAIL;
  }
  return flag;
}

// -------------------------------------------------------------
// Function: check_u_compat
//
// Purpose: check that kinematics and u arrays are compatible.
//
// Arguments:
//   kine   = kinematics array
//   u      = setpt array 
//
// Return: SUCCESS or FAIL
//
// -------------------------------------------------------------
extern int check_u_compat(array_t kine, array_t u)
{
    int flag = SUCCESS;
    if ((kine.nrow != u.nrow)) {
        PRINT_ERR_MSG("u array incompatible with kinematics array");
        flag = FAIL;
    }
    return flag;
}


// ------------------------------------------------------------
// Function: check_motor_cal
//
// Purpose: check the motor calibration satisfies the required
// assumptions. Specifically ...
//
// -------------------------------------------------------------
int check_motor_cal(motor_cal_t motor_cal)
{
  int i;
  int size;
  int flag = SUCCESS; 
  double deg0;
  double deg1;

  // Specific checks based on calibration type
  switch(motor_cal.type) {

    case MOTOR_CALTYPE_TBL:

      // --------------------------------------------
      // Checks specific to lookup table calibrations
      // --------------------------------------------

      // Test that deg_data and ind_data have the correct shape
      if (motor_cal.deg_data.ncol != 1) {
        PRINT_ERR_MSG("bad motor calibration,  deg_data.ncol != 1");
        flag = FAIL;
      }
      if (motor_cal.ind_data.ncol != 1) {
        PRINT_ERR_MSG("bad motor calibration, ind_data.ncol != 1");
        flag = FAIL;
      }

      // Test that deg_data and ind_data are the same size
      if (motor_cal.deg_data.nrow != motor_cal.ind_data.nrow) {
        PRINT_ERR_MSG("bad motor calibration, deg_data.nrow != ind_data.nrow");
        flag = FAIL;
      }
      size = motor_cal.deg_data.nrow;

      // Test for unique ascending values in deg_data array
      for (i=1; i<size; i++) {

        // Read ith value from deg data array
        if (get_array_val(motor_cal.deg_data,i,0,&deg1) != SUCCESS) {
          PRINT_ERR_MSG("get_array_val failed");
          flag = FAIL;
          break;
        }
        // Read i-1th value from deg data array
        if (get_array_val(motor_cal.deg_data,i-1,0,&deg0) != SUCCESS) {
          PRINT_ERR_MSG("get_array_val failed");
          flag = FAIL;
          break;
        }

        if (deg1 < deg0) {
          PRINT_ERR_MSG("bad motor calibration, deg_data not increasing");
          flag = FAIL;
          break;
        }
        else if (deg1 == deg0) {
          PRINT_ERR_MSG("bad motor calibration, deg_data not unique");
          flag = FAIL;
          break;
        }
      }
      break;

    case MOTOR_CALTYPE_MUL:

      // ---------------------------------------------------
      // Checks specific to multiplicative type calibrations
      // ---------------------------------------------------

      if (motor_cal.deg_per_ind == 0.0) {
        PRINT_ERR_MSG("degrees per index == 0.0");
        flag = FAIL;
      }
      break;

    default:
      PRINT_ERR_MSG("unknown calibration type");
      flag = FAIL;
      break;

  } // End switch(motor_cal.type)


  return flag;
}

