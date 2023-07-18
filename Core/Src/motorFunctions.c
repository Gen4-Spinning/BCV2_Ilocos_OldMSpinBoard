
#include "main.h"
#include "stm32f4xx_hal.h"
#include "logicDefines.h"
#include "encoder.h" 
#include "Initialize.h"
#include "functionDefines.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "stdlib.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern int testPwmValue;
extern int filter1;
extern char MOTORARRAY[];
extern char MOTORARRAY_HMI[];
extern int overloadCount[];

int errorCount = 0;
char motorOnSignal = 0;
int delta = 0;

extern  float TD_modifier ;
extern float TD_used ;
extern float req_coilerTG_RPM;
extern float req_GB_outRPM ;

void AllTimerOn(void)
{
		MotorTimer(MOTOR1_TIMER_ON);
		MotorTimer(MOTOR2_TIMER_ON);
		MotorTimer(MOTOR3_TIMER_ON);
		MotorTimer(MOTOR4_TIMER_ON);
		MotorTimer(MOTOR5_TIMER_ON);
		MotorTimer(MOTOR6_TIMER_ON);
		MotorTimer(MOTOR7_TIMER_ON);
		MotorTimer(MOTOR8_TIMER_ON);
}



void UpdateMotorVoltage(uint8_t motorIndex,int pwmValue)
{
	char deltaPwm = 5;
/*if (motorIndex == 1)
	{
		deltaPwm = 1;
	}*/
	
		// needs to be here so that when we run diagnostics we can print these values.
	M[motorIndex].presentRpm = FilterRpm(motorIndex); 
	//------------------------------------------
	
	
	if (M[motorIndex].pwm < pwmValue)
	{
		if (M[motorIndex].pwm < MAX_PWM)
		{
			M[motorIndex].pwm = M[motorIndex].pwm + deltaPwm;
		}
	}
	if (M[motorIndex].pwm > pwmValue)
	{
		if (M[motorIndex].pwm > MIN_PWM)
		{
			M[motorIndex].pwm = M[motorIndex].pwm - deltaPwm;
		}
	}
}

void UpdateMotorPiecing(uint8_t motorIndex)
{
	unsigned int calcPwm = 0;
	M[motorIndex].presentRpm = FilterRpm(motorIndex);
	
	if(M[motorIndex].intTarget <= M[motorIndex].piecingRpm)
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget + M[motorIndex].rampRpm;
	}
	
	if(M[motorIndex].intTarget > M[motorIndex].piecingRpm)
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget - M[motorIndex].rampRpm;
	}
	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;
//	M[motorIndex].error = M[motorIndex].intTarget - filter1;
	
	/*
	if (__fabs(M[motorIndex].error) >= 200)
	{
		E.RpmErrorFlag = 1;
		S.errStopReason = MOTORARRAY[motorIndex];
		S.errmotorFault = ERR_RPM_ERROR;	
		S.errVal = NO_FLOAT_VAR;
	}
	*/
	
	M[motorIndex].integralError = (M[motorIndex].integralError + M[motorIndex].error);
	M[motorIndex].derivativeError = (M[motorIndex].error  - M[motorIndex].preError);
	
	calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError + M[motorIndex].Kd*M[motorIndex].derivativeError;
	
	M[motorIndex].preError = M[motorIndex].error;
	if (calcPwm <= MAX_PWM)
	{
		M[motorIndex].pwm = calcPwm;
	}
}

void ResetSecondaryMotorSteadyStates(){
		M[MOTOR3].steadyState = 0;
		M[MOTOR4].steadyState = 0;
		M[MOTOR5].steadyState = 0;
		M[MOTOR6].steadyState = 0;
		M[MOTOR7].steadyState = 0;
		M[MOTOR8].steadyState = 0;
}

void CheckSteadyStateReached(uint8_t motorIndex){
	if (M[motorIndex].steadyState != 1){
		if (abs(M[motorIndex].intTarget  -  M[motorIndex].setRpm)< 50 ){  // LAter fix the red line near the abs
				M[motorIndex].steadyState = 1;
			}
	}
}

 void UpdateMotorPID(uint8_t motorIndex)
{
	unsigned int calcPwm = 0;
	M[motorIndex].presentRpm = FilterRpm(motorIndex);
	
	if(M[motorIndex].intTarget <= M[motorIndex].setRpm)
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget + M[motorIndex].rampRpm;
		
		// so that Target reaches finalSetRPM and stays there.
		if (M[motorIndex].intTarget > M[motorIndex].setRpm){
			M[motorIndex].intTarget = M[motorIndex].setRpm;
		}
			
	}
	
	if(M[motorIndex].intTarget > M[motorIndex].setRpm)
	{
		M[motorIndex].intTarget = M[motorIndex].intTarget - M[motorIndex].rampRpm;
		
		// so that Target reaches finalSetRPM and stays there.
		if (M[motorIndex].intTarget < M[motorIndex].setRpm){
			M[motorIndex].intTarget = M[motorIndex].setRpm;
		}	
	}
	
	M[motorIndex].error = M[motorIndex].intTarget - M[motorIndex].presentRpm;
	
//	M[motorIndex].error = M[motorIndex].intTarget - filter1;
	
	//0.1 sec per call, so 10 per sec, and 50 in 5 sec
	// for nepal  no overload checking on the conveyor motor.
	if (motorIndex != 7){
		if (M[motorIndex].steadyState == 1){  // only look for error when your in the steadystate region 
			if (abs(M[motorIndex].error) >= M[motorIndex].overloadDelta){
							// we want a time of 5 sec
							overloadCount[motorIndex] ++ ; // only this is an array, maybe we can put this in the motor struct also
							if (overloadCount[motorIndex] > M[motorIndex].overloadTime){
								E.RpmErrorFlag = 1;
								S.errStopReason = MOTORARRAY[motorIndex];
								S.errStopReasonHMI = MOTORARRAY_HMI[motorIndex];
								S.errmotorFault = ERR_RPM_ERROR;	
								S.errVal = NO_FLOAT_VAR;
								overloadCount[motorIndex] = 0; // you can set it to zero because E.rpm error has been set
							}
						}
					else{
					overloadCount[motorIndex] = 0;
				}
			}
		else{ // during rampup and rampdown
				if (abs(M[motorIndex].error) >= 500){ //for all motors a 2 sec region where they show a delta rpm of 300
						errorCount++;
						if(errorCount >= 30){
							E.RpmErrorFlag = 1;
							S.errStopReason = MOTORARRAY[motorIndex];
							S.errStopReasonHMI = MOTORARRAY_HMI[motorIndex];
							S.errmotorFault = ERR_RPM_ERROR;	
							S.errVal = NO_FLOAT_VAR;
							errorCount = 0;
							}		
					}
				}
			} // closes if not MotorIndex = 7


	M[motorIndex].integralError = (M[motorIndex].integralError + M[motorIndex].error);
	M[motorIndex].derivativeError = (M[motorIndex].error  - M[motorIndex].preError);
	
	calcPwm = M[motorIndex].Kp*M[motorIndex].error + M[motorIndex].Ki*M[motorIndex].integralError + M[motorIndex].Kd*M[motorIndex].derivativeError;
	
	M[motorIndex].preError = M[motorIndex].error;
	if (calcPwm <= MAX_PWM)
	{
		M[motorIndex].pwm = calcPwm;
	}
}

char CheckCylReachedFullSpeed(void){
	uint8_t out = 0;
	//delta = (M[MOTOR1].setRpm) -(M[MOTOR1].presentRpm);
	if (M[MOTOR1].presentRpm >= (M[MOTOR1].setRpm - 150)){
		out = 1;
	}
	return out;
}

void PauseMachine(void)
{
	uint8_t i = 0;
	HAL_Delay(100);
	for(i=3;i<6;i++)
		{
			M[i].pwm = 0;
		}
	TowerLamp(GREEN_OFF);
}

void ResetSecondaryMotor(void)
{
				M[MOTOR3].pwm = 0;
				M[MOTOR4].pwm = 0;
				M[MOTOR5].pwm = 0;
				M[MOTOR6].pwm = 0;
				M[MOTOR7].pwm = 0;
				M[MOTOR8].pwm = 0;
				
				M[MOTOR3].intTarget = 0;
				M[MOTOR4].intTarget = 0;
				M[MOTOR5].intTarget = 0;
				M[MOTOR6].intTarget = 0;
				M[MOTOR7].intTarget = 0;
				M[MOTOR8].intTarget = 0;
	
			
				M[MOTOR3].error = 0;
				M[MOTOR4].error = 0;
				M[MOTOR5].error = 0;
				M[MOTOR6].error = 0;
				M[MOTOR7].error = 0;
				M[MOTOR8].error = 0;
				
				M[MOTOR3].integralError = 0;
				M[MOTOR4].integralError = 0;
				M[MOTOR5].integralError = 0;
				M[MOTOR6].integralError = 0;
				M[MOTOR7].integralError = 0;
				M[MOTOR8].integralError = 0;
				
				M[MOTOR3].derivativeError = 0;
				M[MOTOR4].derivativeError = 0;
				M[MOTOR5].derivativeError = 0;
				M[MOTOR6].derivativeError = 0;
				M[MOTOR7].derivativeError = 0;
				M[MOTOR8].derivativeError = 0;
}



void MotorTimer(char index)
{
		switch(index)
	{
		case MOTOR1_TIMER_ON:
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
		break;
		
		case MOTOR1_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
		break;
		
		case MOTOR2_TIMER_ON:
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		break;
		
		case MOTOR2_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		break;
		
		case MOTOR3_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		break;
		
		case MOTOR3_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		break;
		
		case MOTOR4_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		break;
		
		case MOTOR4_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		break;
		
		case MOTOR5_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		break;
		
		case MOTOR5_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		break;
		
		case MOTOR6_TIMER_ON:
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		break;
		
		case MOTOR6_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
		break;
		
		case MOTOR7_TIMER_ON:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
		break;
		
		case MOTOR7_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
		break;
		
		case MOTOR8_TIMER_ON:
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
		break;
		
		case MOTOR8_TIMER_OFF:
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
		break;
	}
}

void KeepFeedMotorsOFF(void){

	M[MOTOR6].presentRpm = 0;
	M[MOTOR6].error = 0;
	M[MOTOR6].integralError = 0;
	M[MOTOR6].intTarget = 0;
	M[MOTOR6].pwm = 0;
	
	M[MOTOR7].presentRpm = 0;
	M[MOTOR7].error = 0;
	M[MOTOR7].integralError = 0;
	M[MOTOR7].intTarget = 0;
	M[MOTOR7].pwm = 0;

}

void ResetFeedMotors(void){
	M[MOTOR6].setRpm = (int)csp.beaterFeed*180.0;
	M[MOTOR7].setRpm = (int)((float)(csp.beaterFeed)* CONVEYOR_MULTIPLIER_CONSTANT)*180.0;
}

void RecalculateTargetRPMs(void){
	//cylinder and beater are not reset
	M[MOTOR3].setRpm = (int)(csp.deliverySpeed*TG_TO_QUIETING_RATIO*ILOCOS_CAGE_GB_RATIO);
	TD_modifier = (csp.tensionDraft-10)*DELTA_TENSION_DRAFT; //ten is 1,11 needs to be 1.08 , every delta from 10 is delta of 0.05
	TD_used =  DEFAULT_TENSION_DRAFT  + TD_modifier;
	req_coilerTG_RPM = csp.deliverySpeed*1000*TD_used/COILER_TG_CIRCUMFERENCE;
	req_GB_outRPM = req_coilerTG_RPM / COILER_TG_TO_MOTOR_GB_RATIO;
	M[MOTOR4].setRpm = (int)(req_GB_outRPM*ILOCOS_COILER_GB_RATIO);
	M[MOTOR5].setRpm = (int)csp.cylinderFeed*180.0;
	M[MOTOR6].setRpm = (int)csp.beaterFeed*180.0;
	M[MOTOR7].setRpm = (int)((float)(csp.beaterFeed)* CONVEYOR_MULTIPLIER_CONSTANT)*180.0;
}

void RecalculateTargetRPMsForPiecing(void){
	//cylinder and beater are not reset
	//delivery speed is 8, feeds are 2.5, converyor gets it speed from btr feed, and tension draft is 1.03, the default
	uint8_t deliverySpeed = 8;
	M[MOTOR3].setRpm = (int)(deliverySpeed*TG_TO_QUIETING_RATIO*ILOCOS_CAGE_GB_RATIO);
	TD_used =  DEFAULT_TENSION_DRAFT;
	req_coilerTG_RPM = csp.deliverySpeed*1000*TD_used/COILER_TG_CIRCUMFERENCE;
	req_GB_outRPM = req_coilerTG_RPM / COILER_TG_TO_MOTOR_GB_RATIO;
	M[MOTOR4].setRpm = (int)(req_GB_outRPM*ILOCOS_COILER_GB_RATIO);
	M[MOTOR5].setRpm = (int)2.5*180.0;
	M[MOTOR6].setRpm = (int)2.5*180.0;
	M[MOTOR7].setRpm = (int)((float)2.5 * CONVEYOR_MULTIPLIER_CONSTANT)*180.0;
}


void TimerLow(char index)
{
	switch(index)
	{
		case T2_CH4:
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,0);
		break;
		
		case T2_CH3:
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,0);
		break;
		
		case T1_CH4:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,0);
		break;
		
		case T1_CH3:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,0);
		break;
		
		case T1_CH2:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,0);
		break;
		
		case T1_CH1:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,0);
		break;
		
		case T3_CH4:
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,0);
		break;
		
		case T3_CH3:
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3,0);
		break;
	}
}


void AllSignalVoltageLow(void)
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3,0);
	
}
void ApplyPwms(void)
{
	if ((M[MOTOR1].pwm <= MAX_PWM ) && (M[MOTOR1].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4,M[MOTOR1].pwm);
	}
	
	if ((M[MOTOR2].pwm <= MAX_PWM ) && (M[MOTOR2].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3,M[MOTOR2].pwm);
	}
	
	if ((M[MOTOR3].pwm <= MAX_PWM ) && (M[MOTOR3].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,M[MOTOR3].pwm);
	}
	
	if ((M[MOTOR4].pwm <= MAX_PWM ) && (M[MOTOR4].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,M[MOTOR4].pwm);
	}
	
	if ((M[MOTOR5].pwm <= MAX_PWM ) && (M[MOTOR5].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,M[MOTOR5].pwm);
	}
	
	if ((M[MOTOR6].pwm <= MAX_PWM ) && (M[MOTOR6].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,M[MOTOR6].pwm);
	}
	
	if ((M[MOTOR7].pwm <= MAX_PWM ) && (M[MOTOR7].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,M[MOTOR7].pwm);
	}
	
	if ((M[MOTOR8].pwm <= MAX_PWM ) && (M[MOTOR8].pwm >= MIN_PWM))
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3,M[MOTOR8].pwm);
	}
	
}


void StopNonCylMotors(void){
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,0);

}


