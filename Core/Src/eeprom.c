
#include "eeprom.h"
#include "Structs.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;
extern float currentLength;

void EepromWriteInt(unsigned position,unsigned int data)
{
	int count = 0;
	if (data>255)
	{
		while(data>255)
		{
			data = data-255;
			count++;
		}
	}
	
	if(count>0)
	{
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position,0xFF,(uint8_t*)&count,1,1);
		HAL_Delay(5);
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position+1,0xFF,(uint8_t*)&data,1,1);
		HAL_Delay(5);
	}
	else
	{	
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position,0xFF,(uint8_t*)&count,1,1);
		HAL_Delay(5);
		HAL_I2C_Mem_Write(&hi2c3,EEPROM_ADDRESS, position+1,0xFF,(uint8_t*)&data,1,1);
		HAL_Delay(5);
	}

}

unsigned int EepromReadInt(unsigned position)
{
	int count =0;
	unsigned int data = 0;
	HAL_I2C_Mem_Read(&hi2c3,EEPROM_ADDRESS,position,0xFF,(uint8_t*)&count,1,1);
	HAL_I2C_Mem_Read(&hi2c3,EEPROM_ADDRESS,position+1,0xFF,(uint8_t*)&data,1,1);
	data = data+(count*255);
	return data;
}


void EepromWriteFloat(unsigned position,float data)
{
	unsigned int data_in = 0;
	int count3 = 0;
	int count2 = 0;
	int count1 = 0;
	data_in =  *(unsigned int*)&data;
	if (data_in>16777216)
	{
		while(data_in>16777216)
		{
			data_in = data_in-16777216;
			count3++;
		}
	}
		if (data_in>65536)
	{
		while(data_in>65536)
		{
			data_in = data_in-65536;
			count2++;
		}
	}
	if (data_in>255)
	{
		while(data_in>255)
		{
			data_in = data_in-255;
			count1++;
		}
	}
	
	EepromWriteInt(position,count3);
	EepromWriteInt(position+2,count2);
	EepromWriteInt(position+4,count1);
	EepromWriteInt(position+6,data_in);
}


unsigned int EepromReadFloat(unsigned position)
{
	long data = 0;
	unsigned int count3_in = 0;
	unsigned int count2_in = 0;
	unsigned int count1_in = 0;
	unsigned int count0_in = 0;
	count3_in = EepromReadInt(position);
	count2_in = EepromReadInt(position+2);
	count1_in = EepromReadInt(position+4);
	count0_in = EepromReadInt(position+6);
	data = count0_in+(count1_in*255)+(count2_in*65536)+(count3_in*16777216);
	return data;  //data has to be converted to float use this *(float*)&data
}

void ReadSettingsFromEeprom(void)
{
	long data_out = 0;
	csp.deliverySpeed = EepromReadInt(DELIVERY_SPEED_ADDR);
	data_out = EepromReadInt(TENSION_DRAFT_ADDR);
	csp.tensionDraft= ((float)data_out)/(float)100;
	csp.cylinderSpeed = EepromReadInt(CYLINDER_SPEED_ADDR);
	data_out = EepromReadInt(CYLINDER_FEED_ADDR);
	csp.cylinderFeed= ((float)data_out)/(float)100;
	csp.beaterSpeed = EepromReadInt(BEATER_SPEED_ADDR);
	data_out = EepromReadInt(BEATER_FEED_ADDR);
	csp.beaterFeed = ((float)data_out)/(float)100;
	data_out	= EepromReadInt(CONVEYOR_SPEED_ADDR);
	csp.conveyorSpeed = ((float)data_out)/(float)100;
	csp.trunkDelay = EepromReadInt(TRUNK_DELAY_ADDR);
	csp.lengthLimit = EepromReadInt(LENGTH_LIMIT_ADDR);
	data_out	= EepromReadInt(LENGTH_CORRECTION_ADDR);
	csp.lengthCorrection = ((float)data_out)/(float)100;	
}

void ReadCurrentLengthFromEeprom(void)
{
	long data_out = 0;
	data_out = EepromReadInt(CURRENT_LENGTH_ADDR);
	currentLength = ((float)data_out)/(float)100;
}

void SaveCurrentLengthIntoEeprom(void)
{
	EepromWriteInt(CURRENT_LENGTH_ADDR,(int)(currentLength*100));
}

void WriteSettingsIntoEeprom(void)
{

	EepromWriteInt(DELIVERY_SPEED_ADDR,csp.deliverySpeed);
	EepromWriteInt(TENSION_DRAFT_ADDR,(int)(csp.tensionDraft*100));
	EepromWriteInt(CYLINDER_SPEED_ADDR,csp.cylinderSpeed);
	EepromWriteInt(CYLINDER_FEED_ADDR,(int)(csp.cylinderFeed*100));
	EepromWriteInt(BEATER_SPEED_ADDR,csp.beaterSpeed);
	EepromWriteInt(BEATER_FEED_ADDR,(int)(csp.beaterFeed*100));
	EepromWriteInt(CONVEYOR_SPEED_ADDR,(int)(csp.conveyorSpeed*100));
	EepromWriteInt(TRUNK_DELAY_ADDR,csp.trunkDelay);
	EepromWriteInt(LENGTH_LIMIT_ADDR,csp.lengthLimit);
	EepromWriteInt(LENGTH_CORRECTION_ADDR,(int)(csp.lengthCorrection * 100));
}

