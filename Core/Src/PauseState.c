#include "StateFns.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include "stm32f4xx_hal.h"
#include "logicDefines.h"
#include "functionDefines.h"
#include "encoder.h"
#include "eeprom.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim7;
extern uint8_t BufferRec[];
extern char BufferTransmit[];

extern int allMotorsOn;
extern int pushbuttonPress;
extern int stopSecondaryMotor;
extern int sensorCut;

extern int lengthOver;
extern float currentLength;
extern float currentLength_corrected ;

void PauseState(void)
{ 
	char sizeofPacket = 0;
	char keypress = 0;
	while(1){
		// and start the timer that signals when to send the data.
		if (HAL_TIM_Base_GetState(&htim7) ==  HAL_TIM_STATE_READY){
			HAL_TIM_Base_Start_IT(&htim7); // currently at 1 sec
		}
						
		if (S.updateBasePackets == 1){
			// keep it here because this will run once and we want this buffer packet to get created only once.
			// also prepare the Run packet Structure
			UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_SCREEN_DATA,HMI_STOP_SCREEN,HMI_STOP_PAUSE,HMI_STOP_PACKET_LENGTH,3);
			UpdateStopPacket(HMI_STOP_REASON_CODE,NO_VAR,NO_VAR);
		    S.updateBasePackets = 0;
		}

		//send Pause packet
		if ((U.TXcomplete ==1) && (U.TXtransfer == 1)){
			 sizeofPacket = UpdateStopPacketString(BufferTransmit,hsb,hsp,S.errStopReason,S.errmotorFault,S.errVal);
			 HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
			 U.TXcomplete = 0;
			 U.TXtransfer = 0;
		}
						
						
		/******************MOTOR LOGIC*******************/
		//Set whichever motors you want off
		StopNonCylMotors();

		if (lengthOver == 1){
			TowerLamp(GREEN_OFF);
			TowerLamp(RED_OFF);
			TowerLamp(AMBER_OFF);
		}else{
			TowerLamp(GREEN_ON);
			TowerLamp(RED_ON);
			TowerLamp(AMBER_OFF);
		}
						
	  // Go and Save the settings if you need to and come back here
		if (S.saveSettingsOnPause == 1)
		{
			S.current_state =  UPDATESETTINGS;
			S.state_change = TO_UPDATE_SETTINGS; // this is the signal to save the settings
			S.prev_state = PAUSE_STATE;
			HAL_TIM_Base_Stop_IT(&htim7);
			break;
			//S.saveSettings is made zero after the settings are written in the settings file
		}
							

		if (S.saveCurrentLength){
				//Write the current Length value into the eeprom one Time only
				SaveCurrentLengthIntoEeprom();
			  S.saveCurrentLength = 0;
		}
						
		if (S.first_enter == 1) // so that you dont immedietely get another press
		{
			HAL_Delay(2000);
			S.first_enter = 0;
		}
											
		//Wait for a keypress and go back to piecing mode. If you pressed pause in Rampup, we want to go back to ramp up.
		keypress = Pushbutton();
		if ((keypress == 1) && (pushbuttonPress == GO_TO_PIECING))
		{
			// go back to run mode and piecing mode
				S.state_change = TO_RUN;
				S.current_state =  RUN_STATE;
				S.prev_state = PAUSE_STATE;
				S.first_enter = 1;
				S.oneTimeflag = 0;
				S.errStopReason = NO_VAR;
				S.errmotorFault = NO_VAR;
				S.errVal = NO_VAR;
				HAL_TIM_Base_Stop_IT(&htim7);

				if (lengthOver == 1){
					currentLength = 0;
					currentLength_corrected = 0;
					lengthOver = 0;
				}

				ResetSecondaryMotor();
				ResetSecondaryEncoderVariables();
				ResetSecondaryMotorSteadyStates();
				TowerLamp(AMBER_OFF);
				sensorCut = 0;

				//Reset Motor SteadyState bool for the motors that stopped

				HAL_Delay(500);

				// check what the cylinder speed is incase you came in while in rampup state and going back also in ramp up state
				if (allMotorsOn == 0){
					allMotorsOn = CheckCylReachedFullSpeed();
				}
				if (allMotorsOn == 1){
					S.updateBasePackets = 1;
					S.runMode = HMI_RUN_PIECING;
					pushbuttonPress = GO_TO_NORMAL; //in Piecing mode, next press will take us into normal
					RecalculateTargetRPMsForPiecing();
					}
				else{
					S.updateBasePackets = 1;
					S.runMode = HMI_RUN_RAMPUP;
					pushbuttonPress = GO_TO_PAUSE; //come back to pause if you press
					RecalculateTargetRPMs();
					}

				//switch off all the lights
				TowerLamp(GREEN_ON);
				TowerLamp(RED_ON);
				TowerLamp(AMBER_ON);
			  break;

		}
						

		//Check for RPM ERROR, to go into halt State(if theres an error in the cyl and btr)
		if (E.RpmErrorFlag == 1)
			{ S.state_change = TO_HALT;
			S.current_state = HALT_STATE;
			S.prev_state = RUN_STATE;
			S.first_enter = 1;
			S.oneTimeflag = 0;
			HAL_TIM_Base_Stop_IT(&htim7);
			break;
		}

	} // closes while
}

