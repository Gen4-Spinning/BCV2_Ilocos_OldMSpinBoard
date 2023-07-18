/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#include "HMI_Fns.h"
#include "HMI_Constants.h"
#include "CommonConstants.h"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern char BufferRec[];
extern char BufferTransmit[];
extern int Cap1;
extern int Cap2;
extern int Cap3;
extern int Cap4;
extern int Cap5;
extern int Cap6;
extern int Cap7;
extern int Cap8;
extern int Cap9;
extern int Cap10;
int msgType = 0;
extern float currentLength;
extern float currentLength_corrected; 
int sizeofPacket  = 0;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_0))
	{
		Cap1++; 
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_1))
	{
		Cap2++; 
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_2))
	{
		Cap3++; 
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap3_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_3))
	{
		Cap4++; 
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap4_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_4))
	{
		Cap5++; 
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap5_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_5))
	{
		Cap6++; 
	}
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_6))
	{
		Cap7++; 
	}
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap6_Pin);
  HAL_GPIO_EXTI_IRQHandler(Cap7_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET) 
		{
			 // ABORT THE UART RECEIVE
			 HAL_UART_AbortReceive_IT(&huart1);			
			 
			 msgType = HMI_BasePacket_Decode(BufferRec);
			
			 if ((int)msgType == FROM_HMI_IM_PAIRED)
			 {
				//stop sending any messages
				HAL_TIM_Base_Stop_IT(&htim7);
				U.TXcomplete = 0;
				U.TXtransfer = 0;
				  
				//And then immedietly send the Settings Packet
				//update the transmit buffer with the Settings Data
				UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,HMI_SETTINGS_INFO,NO_VAR,HMI_SETTINGS_PACKET_LEN,1);
				// transmit packet size is a global. always need to update before we run
				//only for Carding, update the settings packet so that you also add the current length limit
				sizeofPacket =HMI_GetCardingMachineSettingsAllPacketString(BufferTransmit,hsb, csp);
								 
			  HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				U.TXcomplete = 0;
				 
				//FIX FOR APP CANT TAKE A PAUSE SCREEN WITHOUT SEEING A IDLE SCREEN
				 
				S.paired = 1;
				S.updateBasePackets = 1; // if we go directly to RUN mode from here we need this flag to get the run screen.is harmless in the IDLE MODE

				 // to goback to idle screen from Diagnostics mode/. Will automatically switch off if motors are running in diag mode
				if ((S.current_state ==  DIAGNOSTIC_STATE) || (S.current_state ==  UPDATESETTINGS)) 
				{
					S.state_change = TO_IDLE;
				}
				
				HAL_TIM_Base_Start_IT(&htim7);
				
			 }
			 
			 // is signal to go to settings from IDLE
			 if ((int)msgType == FROM_HMI_IN_SETTINGS_CHANGE){
							S.state_change = TO_SETTINGS;
					}
					
			 // go from idle to diagnostics state
			 	if ((int)msgType == FROM_HMI_IN_DIAGNOSTICS_SET)
			 {
				  S.state_change = TO_DIAG;
			 }
			 
			 // will be processed inside the Settings state Fn
			 if ((int)msgType == FROM_HMI_UPDATED_SETTINGS)
			 {
				 if (S.current_state == UPDATESETTINGS){
						 S.state_change = TO_UPDATE_SETTINGS;
				 }
				 else{ // if in run state
					   RecalculateTargetRPMs();
						 S.saveSettingsOnPause = 1;
						 // send an update msg
						 // ACK PACKET
						 UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,FROM_HMI_CHANGE_PROCESS_PARAMS,FROM_MC_ACK_UPDATED_SETTINGS,FROM_MC_ACK_PKT_LEN,NO_VAR);
						 sizeofPacket = HMI_GetSettingsACKPacketString(BufferTransmit,hsb);
						 HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
						 U.TXcomplete = 0; 
						 U.TXtransfer = 0;
						 S.updateBasePackets = 1; // since we ve changed the base packets , we need to again set it back
						}
			 }
			 // will go back to idle settings from settings or diagnostics page
			 if ((int)msgType == FROM_HMI_BACK_TO_IDLE)
			 {
				 S.state_change = TO_IDLE;
			 }
			 
			 if ((int)msgType == FROM_DIAG_UPDATED_TEST_DETAILS)
			 {
				  S.state_change = RUN_DIAG_TEST;
			 }
			 
			  if ((int)msgType == FROM_HMI_RESET_LENGTH)
			 {
				  currentLength = 0;
				  currentLength_corrected = 0;
					// ACK PACKET
					UpdateBasePacket_Modes(CURRENT_MACHINE,HMI_BG_DATA,FROM_HMI_CHANGE_PROCESS_PARAMS,FROM_MC_ACK_ZEROED_LENGTH,FROM_MC_ACK_PKT_LEN,NO_VAR);
					sizeofPacket = HMI_GetSettingsACKPacketString(BufferTransmit,hsb);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)&BufferTransmit, sizeofPacket);
				 	U.TXcomplete = 0; 
					U.TXtransfer = 0; 
				  S.updateBasePackets = 1; // since we ve changed the base packets , we need to again set it back
				  S.saveCurrentLength = 1; // if we re in halt or pause when we zero we want to save the value in the eeprom.
																	 // if we re in run this wont do anything, cos we re not checking the flag.
			 }
			 
				
			HAL_UART_Receive_IT(&huart1,(uint8_t *)BufferRec,MAX_PACKET_SIZE_FROM_HMI);	 // start the receive again.
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		
		
		}
	
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13))
	{
		Cap8++; 
	}
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_14))
	{
		Cap9++; 
	}
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15))
	{
		Cap10++; 
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cap8_Pin);
  HAL_GPIO_EXTI_IRQHandler(Cap9_Pin);
  HAL_GPIO_EXTI_IRQHandler(Cap10_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == USART1)
	 { 
		 U.TXcomplete = 1;
	 }

 }


/* USER CODE END 1 */
