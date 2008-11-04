#ifndef INC_CHECK_H_
#define INC_CHECK_H_
#include "yawff.h"

// Check input arguments for yawff function
extern int check_yawff_input(array_t kine, 
			     config_t config, 
			     data_t data);

// Check system configuration structure
extern int check_config(config_t config);

// Check ranges in system configuration
extern int check_ranges(config_t config);

// Check clk/dir configuration
extern int check_clkdir(config_t config);

// Check mapping from kinematics to motors
extern int check_kine_map(config_t config);

// Check kinematics array
extern int check_kine(array_t kine);

// Check compatibility between configuration and kinematics
extern int check_kine_compat(config_t config, array_t kine);

// Check array structure
extern int check_array(array_t array);

// Check data structure
extern int check_data(data_t data);

// Check that kinematics and data structures compatible
extern int check_data_compat(array_t kine, data_t data);

#endif // INC_CHECK_H_
