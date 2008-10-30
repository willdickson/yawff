// ---------------------------------------------------------
// test.h
//
// Definitions for unit testing
//
// ----------------------------------------------------------
#ifndef INC_TEST_H_ 
#define INC_TEST_H_

// Configuration test parameters
#define DEV_NAME "/dev/comedi0"
#define AIN_SUBDEV 0
#define DIO_SUBDEV 2
#define NUM_MOTOR 7
#define YAW_MOTOR 6
#define DIO_CLK {0,2,4,6,8,10,12}
#define DIO_DIR {1,3,5,7,9,11,13}
#define KINE_MAP {0,1,2,3,4,5}
#define YAW_AIN 0
#define YAW_VOLT2TORQ 1.0
#define YAW_INERTIA 1.0
#define YAW_IND2DEG 1.5
#define YAW_TORQ_LIM 0.5
#define YAW_FILT_CUT 2.0
#define DT_NS 300000

// Kinematics test parameters
#define KINE_NROW 100000
#define KINE_NCOL NUM_MOTOR
#define KINE_PERIOD 5 // seconds
#define KINE_AMP 200  // indices

// Boolean values
#define TRUE 1
#define FALSE 0

// Filter test parameters
#define NUM_CYCLE  50
#define TEST_CYCLE_MIN  20
#define TEST_CYCLE_MAX 40
#define F_SINE  5.0
#define NUM_TESTVALS 50
#define F_TEST_MAX (F_SINE*20.0)
#define F_TEST_MIN (F_SINE/20.0)
#define FILT_DT  (1.0/(F_SINE*1000.0))
#define FILT_GAIN_TOL 0.01

// Integrator test parameters
#define MASS 1.0
#define DAMPING 0.0
#define FORCE (-9.81*MASS)
#define INTEG_DT 0.01
#define INTEG_T 1.0
#define POS_INIT 0.0
#define VEL_INIT 50.0
#define INTEG_TOL 1.0e-6

// Function prototypes
extern void init_test_config(config_t *config);
extern int init_test_kine(array_t *kine, config_t config);
extern void free_test_kine(array_t *kine);
extern int init_test_data(data_t *data, int N);
extern void free_test_data(data_t *data);

#endif // INC_TEST_H_ 
