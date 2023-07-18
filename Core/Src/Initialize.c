#include "Structs.h"
#include "CommonConstants.h"
#include "Initialize.h"
#include "logicDefines.h"
#include "HMI_Constants.h"
#include "CommonConstants.h"
#include <string.h>
//Load the structs here and make them extern in the file you want.

float TD_modifier = 0;
float TD_used =  0;
float req_coilerTG_RPM = 0;
float req_GB_outRPM = 0;
void InitializeStateStruct()
{
	S.current_state = IDLE_STATE;
	S.prev_state = OFF_STATE;
	S.paired = 0;
	S.state_change = 0;
	S.first_enter = 1;
	S.oneTimeflag = 0;
	S.errStopReason = NO_VAR;
	S.errmotorFault = NO_VAR;
	S.errVal = NO_VAR;
	S.firstSwitchon = 1;
	S.runMode = HMI_RUN_RAMPUP; 
	S.updateBasePackets = 0;
	S.RPI_idleSettingsTransferOnce = 0;
	S.oneSecTimer = 0;
	S.switchOffFeed = 0;
	S.ductOn = 1; //default
	S.waitTrunkDelayTime = 0;
	S.saveSettingsOnPause = 0;
	S.saveCurrentLength = 0;
}

void InitializeCardingSettings()
{
    csp.deliverySpeed = 10;
    csp.tensionDraft = 5.0;
    csp.cylinderSpeed = 1500;
    csp.cylinderFeed = 3.5;
    csp.beaterSpeed = 930;
    csp.beaterFeed = 2.5;
    csp.conveyorSpeed = 2;
    csp.trunkDelay = 3;
    csp.lengthLimit = 1500;
	  csp.lengthCorrection = 1.0;
}


void InitializeUartStruct()
{
	U.TXcomplete =0;
	U.TXtransfer =0;
	U.RPI_TXcomplete = 0;
}
void InitializeDiagnosticsStruct()
{
	D.motorID = 0;
	D.motorID = 0;
	D.targetSignal=0;
	D.testTime = 0;
	D.typeofTest = 0;
	
}

void MotorStructInit(void)	
{
		strcpy(M[MOTOR1].name,"__CYL") ;
		M[MOTOR1].presentRpm = 0;
		M[MOTOR1].setRpm = csp.cylinderSpeed;
		M[MOTOR1].rampRpm = 2;
		M[MOTOR1].piecingRpm = 300;
		M[MOTOR1].error = 0;
		M[MOTOR1].derivativeError = 0;
		M[MOTOR1].integralError = 0;
		M[MOTOR1].intTarget = 0; // initial int target
		M[MOTOR1].pwm = 0;
		M[MOTOR1].Ki = 0.1;  // 0.19
		M[MOTOR1].Kd = 0;  //0.13
		M[MOTOR1].Kp = 0.2;  //0.65
 		M[MOTOR1].steadyState = 0;
	  M[MOTOR1].overloadDelta = 200; // delta rpm of 300
		M[MOTOR1].overloadTime = 30; // for 5 sec at 0.1 sec interrupt
	
		strcpy(M[MOTOR2].name,"__BTR");
		M[MOTOR2].presentRpm = 0;
		M[MOTOR2].setRpm = csp.beaterSpeed;
		M[MOTOR2].rampRpm = 5;
		M[MOTOR2].piecingRpm = 300;
		M[MOTOR2].error = 0;
		M[MOTOR2].derivativeError = 0;
		M[MOTOR2].integralError = 0;
		M[MOTOR2].intTarget = 0;
		M[MOTOR2].pwm = 0;
		M[MOTOR2].Ki = 0.19;
		M[MOTOR2].Kd = 0.13;
		M[MOTOR2].Kp = 0.65;
		M[MOTOR2].steadyState = 0;
		M[MOTOR2].overloadDelta = 300;
		M[MOTOR2].overloadTime = 50;
		
		strcpy(M[MOTOR3].name,"_CAGE");
		M[MOTOR3].presentRpm = 0; //CAGE
//		M[MOTOR3].setRpm =(int)((((csp.deliverySpeed/(1+csp.tensionDraft/100))*4.5*1.0)*15.5)/COILER_CAGE_K); 
		M[MOTOR3].setRpm =(int)(csp.deliverySpeed*TG_TO_QUIETING_RATIO*ILOCOS_CAGE_GB_RATIO);
		M[MOTOR3].rampRpm = 10; //was 4
		M[MOTOR3].piecingRpm = 300;
		M[MOTOR3].error = 0;
		M[MOTOR3].derivativeError = 0;
		M[MOTOR3].integralError = 0;
		M[MOTOR3].intTarget = 0;
		M[MOTOR3].pwm = 0;
		M[MOTOR3].Ki = 0.4;
		M[MOTOR3].Kd = 0;
		M[MOTOR3].Kp = 0.065;
		M[MOTOR3].steadyState = 0;
	  M[MOTOR3].overloadDelta = 200;
		M[MOTOR3].overloadTime = 100;

		strcpy(M[MOTOR4].name,"COILR");
		M[MOTOR4].presentRpm = 0;//coiler. 
		TD_modifier = (csp.tensionDraft-10)*DELTA_TENSION_DRAFT; //ten is 1,11 needs to be 1.08 , every delta from 10 is delta of 0.05
		TD_used =  DEFAULT_TENSION_DRAFT  + TD_modifier;
		req_coilerTG_RPM = (csp.deliverySpeed*1000.0*TD_used)/COILER_TG_CIRCUMFERENCE;
		req_GB_outRPM = req_coilerTG_RPM / COILER_TG_TO_MOTOR_GB_RATIO;
		M[MOTOR4].setRpm = (int)(req_GB_outRPM*ILOCOS_COILER_GB_RATIO);

		M[MOTOR4].rampRpm = 10; // was 4
		M[MOTOR4].piecingRpm = 300;
		M[MOTOR4].error = 0;
		M[MOTOR4].derivativeError = 0;
		M[MOTOR4].integralError = 0;
		M[MOTOR4].intTarget = 0;
		M[MOTOR4].pwm = 0;
		M[MOTOR4].Ki = 0.4;
		M[MOTOR4].Kd = 0;
		M[MOTOR4].Kp = 0.065;
		M[MOTOR4].steadyState = 0;
		M[MOTOR4].overloadDelta = 200;
		M[MOTOR4].overloadTime = 100;
		
		strcpy(M[MOTOR5].name,"CYLFD");
		M[MOTOR5].presentRpm = 0;
		M[MOTOR5].setRpm = (int)csp.cylinderFeed*180.0;   //changed from 240 to 180
		M[MOTOR5].rampRpm = 5;
		M[MOTOR5].piecingRpm = 300;
		M[MOTOR5].error = 0;
		M[MOTOR5].derivativeError = 0;
		M[MOTOR5].integralError = 0;
		M[MOTOR5].intTarget = 0;
		M[MOTOR5].pwm = 0;
		M[MOTOR5].Ki = 0.4;
		M[MOTOR5].Kd = 0;
		M[MOTOR5].Kp = 0.065;
		M[MOTOR5].steadyState = 0;
		M[MOTOR5].overloadDelta = 200;
		M[MOTOR5].overloadTime = 100;
	
	    strcpy(M[MOTOR6].name,"BTRFD");
		M[MOTOR6].presentRpm = 0;
		M[MOTOR6].setRpm = (int)csp.beaterFeed*180.0;   //changed from 240 to 180
		M[MOTOR6].rampRpm = 5;
		M[MOTOR6].piecingRpm = 300;
		M[MOTOR6].error = 0;
		M[MOTOR6].derivativeError = 0;
		M[MOTOR6].integralError = 0;
		M[MOTOR6].intTarget = 0;
		M[MOTOR6].pwm = 0;
		M[MOTOR6].Ki = 0.4;
		M[MOTOR6].Kd = 0;
		M[MOTOR6].Kp = 0.065;
		M[MOTOR6].steadyState = 0;
		M[MOTOR6].overloadDelta = 200;
		M[MOTOR6].overloadTime = 100;
		
		strcpy(M[MOTOR7].name,"CNVYR");
		M[MOTOR7].presentRpm = 0;
		M[MOTOR7].setRpm = (int)((float)(csp.beaterFeed)* CONVEYOR_MULTIPLIER_CONSTANT)*180.0;
		M[MOTOR7].rampRpm = 5;
		M[MOTOR7].piecingRpm = 300;
		M[MOTOR7].error = 0;
		M[MOTOR7].derivativeError = 0;
		M[MOTOR7].integralError = 0;
		M[MOTOR7].intTarget = 0;
		M[MOTOR7].pwm = 0;
		M[MOTOR7].Ki = 0.2;
		M[MOTOR7].Kd = 0;
		M[MOTOR7].Kp = 0.065;
		M[MOTOR7].steadyState = 0;
		M[MOTOR7].overloadDelta = 200;
		M[MOTOR7].overloadTime = 100;
		
		strcpy(M[MOTOR8].name,"_____");
		M[MOTOR8].presentRpm = 0;
		M[MOTOR8].setRpm = (int)csp.cylinderFeed*240.0;
		M[MOTOR8].rampRpm = 5;
		M[MOTOR8].piecingRpm = 300;
		M[MOTOR8].error = 0;
		M[MOTOR8].derivativeError = 0;
		M[MOTOR8].integralError = 0;
		M[MOTOR8].intTarget = 0;
		M[MOTOR8].pwm = 0;
		M[MOTOR8].Ki = 0.19;
		M[MOTOR8].Kd = 0.13;
		M[MOTOR8].Kp = 0.65;
		M[MOTOR8].steadyState = 0;
	    M[MOTOR8].overloadDelta = 200;
		M[MOTOR8].overloadTime = 100;
	
		
}
