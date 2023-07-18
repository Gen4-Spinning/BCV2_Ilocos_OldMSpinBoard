
#include "main.h"
#include "functionDefines.h"
#include "stm32f4xx_hal.h"
#include "logicDefines.h"
#include "Structs.h"

int sensorBreak = 0;
int sensorCut = 0;
int sensorReset = 0 ;
int sensorZero = 0;

int sensorCount1 = 0;

extern float currentLength_corrected ;
char InputVoltageSense(void)
{
	char signalOut = 0;
	if (HAL_GPIO_ReadPin(GPIOD,Sig5V_Pin) == 1)
	{
		HAL_Delay(5);
		if (HAL_GPIO_ReadPin(GPIOD,Sig5V_Pin) == 1)
		{
			signalOut = 1;
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOD,Sig12V_Pin) == 1)
	{
		HAL_Delay(5);
		if (HAL_GPIO_ReadPin(GPIOD,Sig12V_Pin) == 1)
		{
			signalOut = 1;
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOC,Sig48V_Pin) == 1)
	{
		HAL_Delay(5);
		if (HAL_GPIO_ReadPin(GPIOC,Sig48V_Pin) == 1)
		{
			signalOut = 1;
		}
	}
	else 
		signalOut = 0;
	return signalOut;
}


char InputSensor1(void)
{
	char sensorOut = 0;
	
	if (HAL_GPIO_ReadPin(GPIOD,Inp4_Pin) == 0)
	{
		HAL_Delay(50);
		if (HAL_GPIO_ReadPin(GPIOD,Inp4_Pin) == 0)
		{
		sensorOut = 1;
		}
	}
		
	/*if((HAL_GPIO_ReadPin(GPIOC,Inp2_Pin) != 0) && (sensorCount1 >0))
		{
				sensorCount1--;	
		}
		
		if (sensorCount1 >= 10)
		{
			sensorOut = 1;
			sensorCount1 = 0;
		}*/
		
	return sensorOut;
}


char LengthOverCheck(void)
{
	//Sensor is normally on, and when it goes off we need to trigger
	char sensorOut = 0;
	if (currentLength_corrected >= csp.lengthLimit){
			sensorOut = 1;
	}
	return sensorOut;
}



char FeedOverLoadSensor(void)
{
	//Sensor is normally on, and when it goes off we need to trigger
	char sensorOut = 0;
	if (HAL_GPIO_ReadPin(GPIOD,Inp3_Pin) == 1)
	{
		HAL_Delay(5);
		if (HAL_GPIO_ReadPin(GPIOD,Inp3_Pin) == 1)
		{
			sensorOut = 1;
		}
	}
	return sensorOut;
}

char DuctSensor(void){
	
	//Sensor is normally on, and when it goes off we need to trigger
	char sensorOut = 0;
	if (HAL_GPIO_ReadPin(GPIOB,Inp9_Pin) == 0){
		HAL_Delay(5);
		if (HAL_GPIO_ReadPin(GPIOB,Inp9_Pin) == 0){
			sensorOut = 1;
		}
	}
	return sensorOut;
}

int ductsensorCount = 0;
int CheckDuctBlocked(int ductSensor){
	int out = 0 ;
	if (ductSensor == 1){
		ductsensorCount +=1;
		if (ductsensorCount > 20){
			out = 1;
		 }
	}
	else{
		ductsensorCount = 0;
   }
	return out;
}

char Pushbutton(void)
{
	char key = 0;
	if (HAL_GPIO_ReadPin(GPIOA,Inp1_Pin) == 0)  //temp changed to inp4
	{
		HAL_Delay(100);
		if (HAL_GPIO_ReadPin(GPIOA,Inp1_Pin) == 0)
			{
				key = 1;
			}
	}
	return key;
}



void LedOn(char index)
{
	switch(index)
	{
		case LED1:
			HAL_GPIO_WritePin(GPIOD,Led1_Pin,GPIO_PIN_SET);		
		break;
			
		case LED2:
			HAL_GPIO_WritePin(GPIOD,Led2_Pin,GPIO_PIN_SET);	
		break;
		
		case LED3:
			HAL_GPIO_WritePin(GPIOD,Led3_Pin,GPIO_PIN_SET);	
		break;
	}
}

void LedOff(char index)
{
	switch(index)
	{
		case LED1:
			HAL_GPIO_WritePin(GPIOD,Led1_Pin,GPIO_PIN_RESET);		
		break;
			
		case LED2:
			HAL_GPIO_WritePin(GPIOD,Led2_Pin,GPIO_PIN_RESET);	
		break;
		
		case LED3:
			HAL_GPIO_WritePin(GPIOD,Led3_Pin,GPIO_PIN_RESET);	
		break;
	}
}

void LedToggle(char index)
{
	switch(index)
	{
		case LED1:
			HAL_GPIO_TogglePin(GPIOD,Led1_Pin);	
		break;
			
		case LED2:
			HAL_GPIO_TogglePin(GPIOD,Led2_Pin);
		break;
		
		case LED3:
			HAL_GPIO_TogglePin(GPIOD,Led3_Pin);
		break;
	}
}

void TowerLamp(char index)
{
	switch(index)
	{
		case GREEN_ON:
			HAL_GPIO_WritePin(GPIOB,TowerRed_Pin,GPIO_PIN_SET);  //TowerGreen_Pin
		break;
		
		case GREEN_OFF:
			HAL_GPIO_WritePin(GPIOB,TowerRed_Pin,GPIO_PIN_RESET);
		break;
		
		case AMBER_ON:
			HAL_GPIO_WritePin(GPIOB,TowerGreen_Pin,GPIO_PIN_SET);
		break;
		
		case AMBER_OFF:
			HAL_GPIO_WritePin(GPIOB,TowerGreen_Pin,GPIO_PIN_RESET);
		break;
		
		case RED_ON:
			HAL_GPIO_WritePin(GPIOB,TowerAmber_Pin,GPIO_PIN_SET);
		break;
		
		case RED_OFF:
			HAL_GPIO_WritePin(GPIOB,TowerAmber_Pin,GPIO_PIN_RESET);
		break;
		
	}
}


void Motor(char index)
{
	switch(index)
	{
		case ENABLE_D:
			HAL_GPIO_WritePin(GPIOB,RelayJ15_Pin,GPIO_PIN_RESET);
		break;
		
		case DISABLE_D:
			HAL_GPIO_WritePin(GPIOB,RelayJ15_Pin,GPIO_PIN_SET);
		break;
		
		case BRAKE_ON:
			HAL_GPIO_WritePin(GPIOD,RelayJ17_Pin,GPIO_PIN_RESET);
		break;
		
		case BRAKE_OFF:
			HAL_GPIO_WritePin(GPIOD,RelayJ17_Pin,GPIO_PIN_SET);
		break;
		
		case REVERSE:
			HAL_GPIO_WritePin(GPIOD,RelayJ18_Pin,GPIO_PIN_SET);
		break;
		
		case FORWARD:
			HAL_GPIO_WritePin(GPIOD,RelayJ18_Pin,GPIO_PIN_RESET);
		break;
		
	}
}

char sensorCount = 0;
char sensorOn = 0;

void SensorBlink(void)
{

	if (sensorOn ==0)
		{TowerLamp(AMBER_OFF);
		sensorOn = 1;
		}
		
	else 
		{TowerLamp(AMBER_ON);
		sensorOn = 0;
		}
}

void SliverBreak(void)
				{
				// at the slowest delivery speed,we get 7m/min, roller dia is 0.08m, 8 cm, and circumference = 0.25 m.
				// so slowest rpm = 28, which means per revolution time > 2.15 sec
				// so if we read every 100 ms, every 22 interrupts we should get one tick. give delta, say 25
				sensorBreak ++;
				
				sensorReset = InputSensor1();
				
				if (sensorReset == 0)
						{
							sensorBreak = 0;
						  TowerLamp(AMBER_ON);// actually off
						}
				
				if (sensorBreak >20 ) //|| ( sensorZero > 30))
				{
						SensorBlink();
				}
				
				if (sensorBreak == 50) //|| (sensorZero == 90))
				{sensorCut = 1;
				}
				HAL_Delay(350);
			}








