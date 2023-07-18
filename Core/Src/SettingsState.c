#include "StateFns.h"
#include "Structs.h"
#include "CommonConstants.h"
#include "logicDefines.h"
#include "HMI_Constants.h"
#include "HMI_Fns.h"
#include "stm32f4xx_hal.h"
#include "eeprom.h"
#include "functionDefines.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim7;
extern uint8_t BufferRec[];
extern char BufferTransmit[];

void SettingsState(void)
{
	int sizeofPacket = 0;
	while(1){

		if (S.state_change == TO_UPDATE_SETTINGS){
				S.state_change = TO_SETTINGS; // so that u update settings only once
				//then write the new values into the eeprom
				WriteSettingsIntoEeprom();
				//Recalculate setRPMs for all motors

				M[MOTOR1].setRpm = csp.cylinderSpeed;
				M[MOTOR2].setRpm = csp.beaterSpeed;
				RecalculateTargetRPMs();

				// ACK PACKET
				UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,FROM_HMI_CHANGE_PROCESS_PARAMS,FROM_MC_ACK_SAVED_SETTINGS,FROM_MC_ACK_PKT_LEN,NO_VAR);
				sizeofPacket = HMI_GetSettingsACKPacketString(BufferTransmit,hsb);
				HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				U.TXcomplete = 0;
				U.TXtransfer = 0;
			}
		
			if (S.prev_state == PAUSE_STATE){
				S.saveSettingsOnPause = 0;
				S.current_state =  PAUSE_STATE;
				S.prev_state = UPDATESETTINGS;
				S.updateBasePackets = 1 ;
				S.first_enter =1;
				break;
			}

		if (S.state_change == TO_IDLE)
			{
				S.current_state =  IDLE_STATE;
				S.prev_state = UPDATESETTINGS;
				S.first_enter = 0; // DONT ALLOW THE RPI TO GET A MESG WHEN U GO BACK FROM SETTINGS
				S.oneTimeflag = 0;
				HAL_TIM_Base_Stop_IT(&htim7);
				break;
			}
		}
}

