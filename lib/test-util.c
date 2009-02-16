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
  test-util.c

  Purpose: Contains functions for setting up unit tests for yawff.h
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#include "yawff.h"
#include "util.h"
#include "test-util.h"

// -----------------------------------------------------------------
// Function: init_config
//
// Purpose: Initialize system configuration
//
// -----------------------------------------------------------------
void init_test_config(config_t *config)
{
  int i;
  int dio_clk[] = DIO_CLK;
  int dio_dir[] = DIO_DIR;
  int kine_map[] = KINE_MAP;

  config -> dev_name = DEV_NAME;
  config -> ain_subdev = AIN_SUBDEV;
  config -> dio_subdev = DIO_SUBDEV;
  config -> num_motor = NUM_MOTOR;
  config -> yaw_motor = YAW_MOTOR;
  for (i=0;i<config->num_motor;i++){
    (config->dio_clk)[i] = dio_clk[i];
    (config->dio_dir)[i] = dio_dir[i];
    if (i<(config->num_motor)-1) {
      (config->kine_map)[i] = kine_map[i]; 
    }
  }
  config -> dio_disable = DIO_DISABLE;
  config -> yaw_ain = YAW_AIN;
  config -> yaw_ain_zero_dt = YAW_AIN_ZERO_DT;
  config -> yaw_ain_zero_num = YAW_AIN_ZERO_NUM;
  config -> yaw_volt2torq = YAW_VOLT2TORQ;
  config -> yaw_inertia = YAW_INERTIA;
  config -> yaw_ind2deg = YAW_IND2DEG;
  config -> yaw_torq_lim = YAW_TORQ_LIM;
  config -> yaw_torq_deadband = YAW_TORQ_DEADBAND;
  config -> yaw_filt_lpcut = YAW_FILT_LPCUT;
  config -> yaw_filt_hpcut = YAW_FILT_HPCUT;
  config -> yaw_damping = YAW_DAMPING;
  config -> dt = DT_NS;
  config -> integ_type = INTEG_RKUTTA;
  return;
}

// ---------------------------------------------------------------
// Function: init_kine
//
// Purpose: Initialize kinematics
//
// ---------------------------------------------------------------
int init_test_kine(array_t *kine, config_t config)
{
  int i;
  int j;
  int pos;
  int flag = SUCCESS;

  if (init_array(kine,KINE_NROW, KINE_NCOL, INT_ARRAY) != SUCCESS) {
    flag = FAIL;
  }
    
  // Create kinematics
  for (i=0; i<kine->nrow; i++) {
    for (j=0; j<kine->ncol; j++) {
      pos = i;
      if(set_array_val(*kine,i,j,&pos)==FAIL) { 
	PRINT_ERR_MSG("error writing kinematic position");
	flag = FAIL;
      }
    }
  }
  return flag;
}

// --------------------------------------------------------------
// Function: free_kine
//
// Purpose: frees memory allocated for kinematics
//
// ---------------------------------------------------------------
void free_test_kine(array_t *kine)
{
  free_array(kine);
}


// ---------------------------------------------------------------
// Function: init_data
//
// Purpose: initize return data structure
//
// ---------------------------------------------------------------
int init_test_data(data_t *data, int N)
{
  int flag = SUCCESS;

  if (init_array(&(data -> t), N, 1, FLT_ARRAY) != SUCCESS) {
    flag = FAIL;
  }
  if (init_array(&(data -> pos), N, 1, FLT_ARRAY) != SUCCESS) {
    flag = FAIL;
  }
  if (init_array(&(data -> vel), N, 1, FLT_ARRAY) != SUCCESS) {
    flag = FAIL;
  }
  if (init_array(&(data -> torq), N, 2, FLT_ARRAY) != SUCCESS) {
    flag = FAIL;
  }
  return flag;
}

// ---------------------------------------------------------------
// Function: free_data
//
// Prupose: fress memory allocated for return data
//
// ---------------------------------------------------------------
void free_test_data(data_t *data)
{
  free_array(&(data -> t));
  free_array(&(data -> pos));
  free_array(&(data -> vel));
  free_array(&(data -> torq));
}
