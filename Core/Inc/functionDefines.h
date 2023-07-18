#ifndef __FUNCTIONDEFINES_H
#define __FUNCTIONDEFINES_H

#include "stdio.h"

void MotorTimer(char index);
void MotorStructInit(void);
void UpdateMotorPID(uint8_t motorIndex);
void UpdateMotorPiecing(uint8_t motorIndex);
void CheckSteadyStateReached(uint8_t motorIndex);
void ResetSecondaryMotorSteadyStates(void);
char RpmError(void);
char CheckCylReachedFullSpeed(void);
void UpdateMotorVoltage(uint8_t motorIndex,int pwmValue);
void AllTimerOn(void);
void ResetSecondaryMotor(void);
void SensorBlink(void);
void SliverBreak(void);
char Pushbutton(void);
char LengthOverCheck(void);
char DuctSensor(void);
char FeedOverLoadSensor(void);
int CheckDuctBlocked(int ductSensor);
void KeepFeedMotorsOFF(void);
void ResetFeedMotors(void);
void RecalculateTargetRPMs(void);

void LedOn(char index);
void LedOff(char index);
void LedToggle(char index); 


char InputSensor1(void);
char InputSensor2(void);
char InputSensor3(void);

void TowerLamp(char index);

void Motor(char index);


char InputVoltageSense(void);
void RunMachine(void);


void ApplyPwms(void);
void AllSignalVoltageLow(void);
void TimerLow(char index);
void StopNonCylMotors(void);
void RecalculateTargetRPMsForPiecing(void);

#endif 
