#ifndef __STRUCTS_H
#define __STRUCTS_H

struct Uart
{
	volatile char TXcomplete ; //Flag that says whether a uart transmission is complete. is set to 1 inside the TX finished interrupt
										// so make it 0 after every transmission call
	volatile char TXtransfer; // Flag used in run mode and diagnosic mode where there is continuous data to link a timer with the logic code.
	volatile char RPI_TXcomplete;
};

struct State 
{	
	char current_state; //current state
	char prev_state;    // prev state
	char paired;				// paired with HMI bool
	char state_change;	// bool to actually force a state change
	char first_enter;		// flag to tell when u enter a state for the first time
	char oneTimeflag;   // bool flag which can be used for any thing  
	char errStopReason;  // err motor ID
	char errStopReasonHMI ;// stop reason with hmi code
	char errmotorFault; // reason for stopping
	float errVal;				// err value
	char firstSwitchon; // state that says when we first switch on the machine
	char runMode;				// the current run state, normal, ramp or piecing.(FOR HMI)
	char updateBasePackets; // flag when u change run state (FOR HMI)
	char RPI_idleSettingsTransferOnce  ; // with the RPI, need to send settings once when in idle the first time.
	int oneSecTimer;
	int switchOffFeed;
	int ductOn;
	int waitTrunkDelayTime;
	int saveSettingsOnPause;
	int saveCurrentLength;
};

struct CardingSettings
{
    int deliverySpeed;
    float tensionDraft;
    int cylinderSpeed;
    float cylinderFeed;
    int beaterSpeed ;
    float beaterFeed;
    float conveyorSpeed;
    int trunkDelay;
    int lengthLimit;
	  float lengthCorrection;
};

struct Diagnostics
{
    int typeofTest;
    int motorID;
    int targetRPM;
    int targetSignal;
    int testTime;
};

struct Motor 
{	
	char name[5] ;
	unsigned int setRpm;	
	unsigned int presentRpm;
	float rampRpm;
	unsigned int piecingRpm;
	signed int intTarget;
	signed int error;
	signed int integralError;
	signed int derivativeError;
	signed int preError;
	unsigned int pwm;
	float Kp;
	float Ki;
	float Kd;
	int steadyState;
	int overloadDelta;
	int overloadTime;
};

// TO keep structs for logic and for communication different
struct Error 
{
	unsigned int RpmErrorFlag;
};

extern struct Motor M[9];
extern struct State S;
extern struct CardingSettings csp;
extern struct Uart U;
extern struct Error E;
extern struct Diagnostics D;
#endif
