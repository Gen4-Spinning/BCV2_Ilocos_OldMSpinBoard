#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_ADDRESS 0xAC//left is 0xa8, right is 0xAC

#define DELIVERY_SPEED_ADDR 0X00
#define TENSION_DRAFT_ADDR 0X02
#define CYLINDER_SPEED_ADDR 0X06
#define CYLINDER_FEED_ADDR 0X08
#define BEATER_SPEED_ADDR 0X0C
#define BEATER_FEED_ADDR 0X0E
#define CONVEYOR_SPEED_ADDR 0X12
#define TRUNK_DELAY_ADDR 0X16
#define LENGTH_LIMIT_ADDR 0X18
#define LENGTH_CORRECTION_ADDR 0x20 // is float
#define CURRENT_LENGTH_ADDR 0x32 // is a float

void EepromWriteInt(unsigned position,unsigned int data);
unsigned int EepromReadInt(unsigned position);
void EepromWriteFloat(unsigned position,float data);
unsigned int EepromReadFloat(unsigned position);
void WriteSettingsIntoEeprom(void);
void ReadSettingsFromEeprom(void);

void ReadCurrentLengthFromEeprom(void);
void SaveCurrentLengthIntoEeprom(void);


#endif /* __ENCODER_H */
