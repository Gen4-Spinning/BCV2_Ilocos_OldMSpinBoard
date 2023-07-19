#include "StateFns.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include "stm32f4xx_hal.h"
#include "functionDefines.h"
#include "logicDefines.h"
#include "encoder.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim7;
extern uint8_t BufferRec[];
extern char BufferTransmit[];
extern int overloadCount[];

//RUN LOGIC variables
extern int allMotorsOn;
extern int pushbuttonPress;
extern int stopSecondaryMotor;
extern int conveyorStart;

extern int Cap7;
extern int Rpm7;
extern int filter7;

extern int sensorCut;
//production
extern int production;
extern float currentLength_corrected;

int feedOverload = 0;
int lengthOver = 0;
int ductSensor = 0;
int switchOffFeed = 0;
int dontComeIn = 0;
uint8_t startTimer = 0;

void RunState(void)
{ 
	char keyPress = 0;
	char sizeofPacket = 0;
	while(1){
		// and start the timer that signals when to send the data.
		if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
			HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
		}
										
		// prepare the Run packet Structure
		if (S.updateBasePackets == 1){
			UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_RUN_SCREEN,S.runMode,HMI_RUN_PACKET_LENGTH,3);
			UpdateRunPacket(INT,HMI_CYL_RPM,899,0,HMI_BTR_RPM,1200,HMI_PRODUCTION,0);
			S.updateBasePackets = 0;
		}

		//send Run packet
		if ((U.TXcomplete ==1) && (U.TXtransfer == 1)){
			 sizeofPacket = UpdateRunPacketString(BufferTransmit,hsb,hrp,M[MOTOR1].presentRpm,M[MOTOR2].presentRpm,(int)currentLength_corrected);
			 HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
			 U.TXcomplete = 0;
			 U.TXtransfer = 0;
		}
						
		if (S.firstSwitchon == 1){
			// Switch on the beater and Cylinder Motors
				ResetEncoderVariables();
				Motor(DISABLE_D);
				MotorTimer(MOTOR1_TIMER_ON);
				MotorTimer(MOTOR2_TIMER_ON);
				TimerLow(T1_CH4);
				TimerLow(T1_CH3);
				TimerLow(T1_CH2);
				TimerLow(T1_CH1);
				TimerLow(T3_CH3);
				// Switch off the secondaryMotors
				ResetSecondaryMotor();
				ResetSecondaryEncoderVariables();

				allMotorsOn = 0; // INITIAL CYLINDER AND BEATER RUN

				S.firstSwitchon = 0;
				S.ductOn = 1;
				S.runMode = HMI_RUN_RAMPUP;
			}

			if (lengthOver == 0){
				if (S.runMode == HMI_RUN_PIECING){
					TowerLamp(GREEN_OFF); // green and amber
					TowerLamp(RED_ON);
					TowerLamp(AMBER_OFF);
				}
				else{
					TowerLamp(GREEN_OFF); // only green
					TowerLamp(RED_ON);
					TowerLamp(AMBER_ON);
				}
			}else{
				TowerLamp(GREEN_OFF);
				TowerLamp(RED_OFF);
				TowerLamp(AMBER_OFF);
			}
						
		  // check if target Rpm is reached by cyl and beater, and switch on the others
		   if (S.runMode == HMI_RUN_RAMPUP){
			  allMotorsOn = CheckCylReachedFullSpeed();
			  if (allMotorsOn == 1){
				  S.updateBasePackets = 1;
				  S.runMode = HMI_RUN_NORMAL;
				  }
			  }

			lengthOver = LengthOverCheck(); //goes to pause
			if (lengthOver == 1){
				//for 5 seconds blink like its over
				if (S.lengthOverTimer < 10){
					//just wait till its over
				}else{
					S.lengthOverTimer = 0;
					currentLength  = 0;
					currentLength_corrected = 0;
					lengthOver = 0;
				}
			}

			//Check for User KeyPress to take machine into Pause State
			keyPress = Pushbutton();

			if ((keyPress==1) && (pushbuttonPress == GO_TO_NORMAL)){ // piecing
				RecalculateTargetRPMs();
				keyPress = 0;
				pushbuttonPress= GO_TO_PAUSE;
				HAL_Delay(500);
				S.updateBasePackets = 1;
				S.runMode = HMI_RUN_NORMAL;
			}

			if (((keyPress==1) && (pushbuttonPress == GO_TO_PAUSE)) || (sensorCut == 1) || (lengthOver == 1)){
				S.state_change = TO_PAUSE;
				S.current_state =  PAUSE_STATE;
				S.prev_state = RUN_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				if (sensorCut == 1)
					{S.errStopReason = ERR_SLIVER_CUT_ERROR;
					 S.errmotorFault = NO_VAR;
					 S.errVal = NO_FLOAT_VAR;
					}
				/*else if (lengthOver == 1){
					S.errStopReason = ERR_LENGTHOVER;
					S.errmotorFault = NO_VAR;
					S.errVal = NO_FLOAT_VAR;
				}*/
				else{
					 S.errStopReason = ERR_USER_PAUSE;
					 S.errmotorFault = NO_VAR;
					 S.errVal = NO_FLOAT_VAR;
					}
				HAL_TIM_Base_Stop_IT(&htim7);

				//update variables for runState changing
				pushbuttonPress = GO_TO_PIECING; // pause mode
				S.runMode = ONLY_CYL_RUNNING;
				ResetSecondaryMotor();
				ResetSecondaryEncoderVariables();
				HAL_Delay(500);
				S.updateBasePackets = 1 ;
				S.saveCurrentLength = 1;
				break;
			}

			feedOverload = FeedOverLoadSensor();
			if (feedOverload==1){
				E.RpmErrorFlag = 1;
				S.errStopReason = ERR_FEEDOVERLOAD;
				S.errStopReasonHMI = ERR_FEEDOVERLOAD;
				S.errmotorFault = NO_VAR;
				S.errVal = NO_FLOAT_VAR;
			}

			ductSensor = DuctSensor();
			S.switchOffFeed = CheckDuctBlocked(ductSensor);
			if (S.ductOn){
				if (S.switchOffFeed){
					if (startTimer == 0){
						S.oneSecTimer = 0;
						startTimer = 1;
					}else{
						if (S.oneSecTimer >= csp.trunkDelay){
							S.ductOn = 0;
							startTimer = 0;
							//turn off happens in tim6 interrupt
						}
					}
				}else{ //might have triggered but not for trunk delay time, so restart the count
					startTimer = 0;
					S.ductOn = 1;
				}
			}else{ // if duct is off, (ductOn = 0)
				if (S.switchOffFeed == 0){
					if (startTimer == 0){
						S.oneSecTimer = 0;
						startTimer = 1;
					}else{
						if (S.oneSecTimer >= csp.trunkDelay){
							S.ductOn = 1;
							startTimer = 0;
							ResetFeedMotors();
						}
					}
				}else{//might have triggered but not for trunk delay time, so restart the count
					startTimer = 0;
					S.ductOn = 0;
				}
			}
						/* Disabled
						if (S.runMode == HMI_RUN_NORMAL)// only look for sliver cut in normal running
						{
						SliverBreak(); // will check if sliver cut and set the variable slivercut = 1
						//CURRENT LENGTH LOGIC
						//until we put the sensor on an interrupt this will be unreliable.. we dont know exactly at what
						//interval this loop is running.. to work around that, we just check the sliver cut variable in the
						//1 sec timer, and accumalate the length within that.							
						}*/
						
					//Check for RPM ERROR, to go into halt State
					if (E.RpmErrorFlag == 1) 
					{ S.state_change = TO_HALT;
						S.current_state = HALT_STATE;
						S.prev_state = RUN_STATE;
						S.first_enter = 1;
						S.oneTimeflag = 0;
						HAL_TIM_Base_Stop_IT(&htim7);
						S.updateBasePackets = 1;
						S.saveCurrentLength = 1;
						break;
					}
	} // closes while
}



