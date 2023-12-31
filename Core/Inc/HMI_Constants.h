
#define HMI_START_END_CHAR  126 // 0x7E

#define MAX_PACKET_SIZE_FROM_HMI 85

//msg type
#define HMI_SCREEN_DATA 0x01
#define HMI_BG_DATA 0x02
//Screens
#define HMI_IDLE_SCREEN 0x01
#define HMI_RUN_SCREEN 0x02
#define HMI_STOP_SCREEN 0x03
// Information
#define HMI_SETTINGS_INFO 0x04
#define HMI_SETTINGS_PACKET_LEN 33 //without the /r in the end
#define HMI_DIAG_RESULTS 0x05 
//DIAG_TEST_Results
#define HMI_DIAG_TEST_SUCCESSFUL 0x98
#define HMI_DIAG_TEST_FAIL 0x97
#define HMI_DIAG_TEST_MOTORERROR 0x96

//SubScreens
#define HMI_RUN_NORMAL 0x01
#define HMI_RUN_RAMPUP 0x02
#define HMI_RUN_PIECING 0x03
#define ONLY_CYL_RUNNING 0x04 // added to not run the control loops for motors in pause state
#define HMI_RUN_PACKET_LENGTH 19
#define HMI_STOP_PAUSE 0x11 
#define HMI_STOP_HALT 0x12
#define HMI_STOP_PACKET_LENGTH 19
#define HMI_IDLE_SCREEN_CYLROTATING 0x89
//Senders
#define MACHINE_TO_HMI 0x01
#define HMI_TO_MACHINE 0x02


//ATTRIBUTE CODES
//1. FOR RUN SCREENS
#define HMI_CYL_RPM  0x01 //INT
#define HMI_BTR_RPM  0x02 //INT
#define HMI_PRODUCTION 0x05

//2. for STOP Screens
#define HMI_STOP_REASON_CODE  0x01
#define HMI_MOTOR_FAULT_CODE 0x02
#define HMI_ERROR_VAL_CODE 0x03
#define HMI_USER_PAUSE 0x99

//3.BLOW CARD MOTORS and error reasons.
//Same as RPI.

//4. DIAG CODES // NOT USED
#define HMI_DIAG_TEST_CODE 0x01
#define HMI_DIAG_MOTOR_ID_CODE 0x02
#define HMI_DIAG_SIGNAL_VOLTAGE_CODE 0x03
#define HMI_DIAG_TARGET_RPM_CODE 0x04
#define HMI_DIAG_TEST_TIME_CODE 0x05
//attibutes
#define HMI_DIAG_CLOSED_LOOP  0x01
#define HMI_DIAG_OPEN_LOOP  0x02

#define HMI_DIAG_END_OF_TEST_CODE 0x0A //RETURN

//5. BLOW CARD PARAMETERS
//same as RPI...

//SETTINGS
#define HMI_CARDING_ALL_SETTING_CODE 0x10
#define HMI_CARDING_ALL_SETTINGS_PKT_LEN 24

// RETURN MSGS FROM HMI TO MACHINE
#define FROM_HMI_IM_PAIRED 0x99
#define IM_PAIRED_PACKET_LENGTH 26  // including the /r
#define FROM_HMI_DISABLE_MACHINE_START 0x03
#define FROM_HMI_ENABLE_MACHINE_START 0x02
#define FROM_HMI_IN_SETTINGS_CHANGE 0x01
#define FROM_HMI_IN_DIAGNOSTICS_SET 0x02
#define FROM_HMI_BACK_TO_IDLE 0x00

#define FROM_HMI_CHANGE_PROCESS_PARAMS 0x04
#define FROM_HMI_CHANGE_HW_PARAMS 0x03 //unused
#define FROM_HMI_DIAGNOSTIC_TEST 0x05 

#define FROM_HMI_RESET_LENGTH 0x0C


//INTERNALLY DEFINED MSGS
#define FROM_HMI_UPDATED_SETTINGS 0x06
#define FROM_MC_ACK_UPDATED_SETTINGS 0x99
#define FROM_MC_ACK_SAVED_SETTINGS 0x88
#define FROM_MC_ACK_ZEROED_LENGTH 0x77
#define FROM_MC_ACK_PKT_LEN 11
#define FROM_DIAG_UPDATED_TEST_DETAILS 0x10
#define RUN_DIAG_TEST 0x12

//MOTOR CODES FOR HMI ONLY
#define HMI_CARD_CYLINDER 0x01
#define HMI_CARD_BEATER 0x02
#define HMI_CARD_CAGE 0x03
#define HMI_CARD_COILER 0x04
#define HMI_CARD_FEED 0x05
#define HMI_CARD_BEATER_FEED 0x06
#define HMI_CARD_CONVEYOR 0x07

