
//Constants common to Logic, and the two Communication channels

//current machine SETTINGs
#define CURRENT_MACHINE CARDING
#define CURRENT_MACHINE_ID 1

//MACHINES
#define CARDING 1
#define DRAWFRAME 2
#define FLYER 3

#define COILER_CAGE_K  1.0 // multiplicative factor used in Nepal for correcting coiler cage rpm ratios

//STATES
#define OFF_STATE 0
#define IDLE_STATE 1
#define DIAGNOSTIC_STATE 2
#define RUN_STATE 3
#define PAUSE_STATE 4
#define HALT_STATE 5
#define UPDATESETTINGS 6 

//change states
#define TO_RUN 0
#define TO_DIAG 1
#define TO_SETTINGS 2
#define TO_PAUSE 3
#define TO_HALT 4
#define TO_UPDATE_SETTINGS 5
#define TO_IDLE 6

//MOTORS
#define CARD_CYLINDER 0x01
#define CARD_BEATER   0x02
#define CARD_CAGE	0x03
#define CARD_COILER	0x04
#define CARD_FEED	0x05
#define CARD_BEATER_FEED 0x06
#define CARD_CONVEYOR 0x07

//DIAG CODES. HMI AND RPI have different test type codes. TO FIX in next round
#define DIAG_ATTR_MOTORID 0x01
#define DIAG_ATTR_SIGNAL_IP_PERCENT 0x02
#define DIAG_ATTR_TARGET_RPM 0x03
#define DIAG_ATTR_TEST_TIME 0x04
#define DIAG_ATTR_TEST_RESULT 0x05

// ERROR CODES
#define ERR_RPM_ERROR 0x02
#define ERR_MOTOR_VOLTAGE_ERROR 0x03
#define ERR_DRIVER_VOLTAGE_ERROR 0x04
#define ERR_SIGNAL_VOLTAGE_ERROR 0x05
#define ERR_OVERLOAD_CURRENT_ERROR 0x06
#define ERR_USER_PAUSE 0x08
#define ERR_SLIVER_CUT_ERROR 0x09
#define ERR_FEEDOVERLOAD 0x0A
#define ERR_LENGTHOVER 0x0B
//DATATYPES
#define FLOAT 4
#define INT 2
#define CHAR 1

#define TG_TO_QUIETING_RATIO 4.58

#define ILOCOS_CAGE_GB_RATIO 15.5
#define ILOCOS_COILER_GB_RATIO 15.5
#define DEFAULT_TENSION_DRAFT 1.1
#define DELTA_TENSION_DRAFT 0.1
#define COILER_TG_CIRCUMFERENCE 194.779
#define COILER_TG_TO_MOTOR_GB_RATIO 1.767
#define CONVEYOR_MULTIPLIER_CONSTANT 0.66


#define GO_TO_PAUSE 0
#define GO_TO_PIECING 1
#define GO_TO_NORMAL 2

//MISC
#define NO_VAR 0x00
#define NO_FLOAT_VAR (float)0.0
	

