#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "stdio.h"
#include "HMI_Fns.h"
#include "HMI_Constants.h"
#include "CommonConstants.h"
#include "stm32f4xx_hal.h"

extern float currentLength;
extern float currentLength_corrected;
extern float deliverySpeed_perSec;

void Create_HMI_BasePacket(void)
{
    hsb.start = HMI_START_END_CHAR;
    hsb.sender = MACHINE_TO_HMI;
    hsb.packetLength = 12; // gets updated
    hsb.machineID = CURRENT_MACHINE_ID;
    hsb.machineType = CURRENT_MACHINE; // gets updated
    hsb.msgType = HMI_SCREEN_DATA; // gets updated
    hsb.nextscreen =  HMI_RUN_SCREEN; // gets updated
    hsb.screen_substate = HMI_RUN_RAMPUP; // gets updated
    hsb.attributeCount = 3; // gets updated
    hsb.endChar = HMI_START_END_CHAR;
}

void Create_HMI_Run_Packet(void)
{
    hrp.tlv1Code = HMI_CYL_RPM;
    hrp.tlv1length = 2;
    hrp.tlv1_val_float = 0;
    hrp.tlv1_val_int = 0;
    hrp.tlv2Code = HMI_BTR_RPM;
    hrp.tlv2length = 2;
    hrp.tlv2_val = 0;
    hrp.tlv3Code = NO_VAR;
    hrp.tlv3length = 2;
    hrp.tlv3_val = 0;

}

void Create_HMI_StopPacket(void)
{
    hsp.tlv1Code = HMI_STOP_REASON_CODE;
    hsp.tlv1length = 2;
    hsp.tlv1_val = CARD_BEATER;
    hsp.tlv2Code = HMI_MOTOR_FAULT_CODE;
    hsp.tlv2length = 2;
    hsp.tlv2_val = ERR_RPM_ERROR;
    hsp.tlv3Code = HMI_ERROR_VAL_CODE;
    hsp.tlv3length =4;
    hsp.tlv3_val = 40;
};

void Create_HMI_DiagPacket(void)
{
    hdp.tlv1Code = HMI_DIAG_TEST_CODE;
    hdp.tlv1length = 2;
    hdp.tlv1_val = 0;
    hdp.tlv2Code = HMI_DIAG_MOTOR_ID_CODE;
    hdp.tlv2length = 2;
    hdp.tlv2_val = 0;
    hdp.tlv3Code = HMI_DIAG_SIGNAL_VOLTAGE_CODE;
    hdp.tlv3length = 2;
    hdp.tlv3_val = 0;
    hdp.tlv4Code = HMI_DIAG_TEST_TIME_CODE;
    hdp.tlv4length = 2;
    hdp.tlv4_val = 0;
}

void UpdateBasePacket_Modes(char machineType,char msgType,char state,char runsubMode,char packetLength,char attributeCount)
{
    hsb.msgType = msgType;
    hsb.machineType = machineType;
    hsb.nextscreen = state;
    hsb.screen_substate = runsubMode;
    hsb.packetLength = packetLength;
    hsb.attributeCount = attributeCount;
}

void UpdateRunPacket(char tlv1Type,char tlv1Code,int tlv1Val_i,float tlv1Val_f,char tlv2Code,int tlv2Val,char tlv3Code,int tlv3Val)
{
    hrp.tlv1Code = tlv1Code;
    if (tlv1Type == INT)
        {
         hrp.tlv1length = 2;
         hrp.tlv1_val_int = tlv1Val_i;
         hrp.tlv1_val_float =0;
        }
    else{
         hrp.tlv1length = 4;
         hrp.tlv1_val_int = 0;
         hrp.tlv1_val_float = tlv1Val_f;
    }
    hrp.tlv2Code = tlv2Code;
    hrp.tlv2_val = tlv2Val;
    hrp.tlv3Code = tlv3Code;
    hrp.tlv3_val = tlv3Val;

  
}


void UpdateStopPacket(char tlv1Code,char tlv2Code,char tlv3Code)
{
    hsp.tlv1Code = tlv1Code;
    hsp.tlv2Code = tlv2Code;
    hsp.tlv3Code = tlv3Code;
		hsp.tlv3length = 2;
		
}


int UpdateRunPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_RunPacket hrp,int cyl_RPM,int btr_RPM,int production)
{    int sizeofPacket = 0;
     if (hrp.tlv1length ==2) // if tlv1 is an int
     {
         sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%04X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
         hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hrp.tlv1Code,
         hrp.tlv1length,cyl_RPM,hrp.tlv2Code,hrp.tlv2length,btr_RPM,hrp.tlv3Code,hrp.tlv3length,
         production,hsb.endChar);
         sizeofPacket = 45;
    }
    else{ //if its a float
         sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%X%02X%02X%04X%02X%02X%04X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
         hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hrp.tlv1Code,
         hrp.tlv1length,*(unsigned int*)&production,hrp.tlv2Code,hrp.tlv2length,hrp.tlv2_val,hrp.tlv3Code,hrp.tlv3length,
         hrp.tlv3_val,hsb.endChar);
         sizeofPacket = 50;
    }
    return sizeofPacket;
}


int UpdateStopPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,float error_val)
{   int sizeofPacket = 0;
	  if (error_val != NO_FLOAT_VAR)
		{
		sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hsp.tlv1Code,
		hsp.tlv1length,stop_reason,hsp.tlv2Code,hsp.tlv2length,motor_fault,hsp.tlv3Code,hsp.tlv3length,
		*(unsigned int*)&error_val,hsb.endChar);
		}
		else
		{
			sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%08X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
			hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hsp.tlv1Code,
			hsp.tlv1length,stop_reason,hsp.tlv2Code,hsp.tlv2length,motor_fault,hsp.tlv3Code,hsp.tlv3length,
			*(unsigned int*)&error_val,hsb.endChar);
		}
		sizeofPacket = 49;
		
    return sizeofPacket;
}

int UpdatePausePacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,float error_val)
{   int sizeofPacket = 0;
	
		sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%08X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hsp.tlv1Code,
		hsp.tlv1length,stop_reason,hsp.tlv2Code,hsp.tlv2length,motor_fault,hsp.tlv3Code,hsp.tlv3length,
		*(unsigned int*)&error_val,hsb.endChar);
		sizeofPacket = 49;
		
    return sizeofPacket;
}

int HMI_GetCardingMachineSettingsAllPacketString(char *buffer,struct HMI_InfoBase hsb,struct CardingSettings csp)
{ int sizeofPacket = 0;
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02d%04X%X%04X%X%04X%X%X%04X%04X%X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         HMI_CARDING_ALL_SETTING_CODE,HMI_CARDING_ALL_SETTINGS_PKT_LEN,csp.deliverySpeed,*(unsigned int*)&csp.tensionDraft,
				 csp.cylinderSpeed,*(unsigned int*)&csp.cylinderFeed,csp.beaterSpeed,*(unsigned int*)&csp.beaterFeed,
				 *(unsigned int*)&csp.conveyorSpeed,csp.trunkDelay,csp.lengthLimit,*(unsigned int*)&csp.lengthCorrection,
					 (int)currentLength_corrected,hsb.endChar);
  sizeofPacket = 89;
  return sizeofPacket;
}

int HMI_GetIdlePacketString(char *buffer,struct HMI_InfoBase hsb)
{	int sizeofPacket = 0;
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         hsb.endChar);
  sizeofPacket = 21;
  return sizeofPacket;
	
}

int HMI_GetSettingsACKPacketString(char *buffer,struct HMI_InfoBase hsb)
{	int sizeofPacket = 0;
	
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,
         0,0,0,hsb.endChar);
  sizeofPacket = 29;
  return sizeofPacket;
	
}

int UpdateDiagPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_DiagPacket hdp,int typeOfTest,int motorID,int signalVoltage,int rpm)
{    int sizeofPacket = 0;
    //tlv1-type of test,tlv2 - motorID .tlv3 - sigal voltage ,tlv4 - current trpm
		sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X%02X%04X%02X%02X%04X%02X%02X%04X%02X\r",hsb.start,hsb.sender,hsb.packetLength,
		hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,hsb.screen_substate,hsb.attributeCount,hdp.tlv1Code,
		hdp.tlv1length,typeOfTest,hdp.tlv2Code,hdp.tlv2length,motorID,hdp.tlv3Code,hdp.tlv3length,
    signalVoltage,hdp.tlv4Code,hdp.tlv4length,rpm,hsb.endChar);
    sizeofPacket = 53;
    
    return sizeofPacket;
}

int HMI_Get_DiagOver_PacketString(char *buffer,struct HMI_InfoBase hsb,char status)
{	int sizeofPacket = 0;
	
  sprintf(buffer,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%04X%02X\r",hsb.start,hsb.sender,
         hsb.packetLength,hsb.machineID,hsb.machineType,hsb.msgType,hsb.nextscreen,status,1,
         HMI_DIAG_END_OF_TEST_CODE,2,0,hsb.endChar);
  sizeofPacket = 29;
  return sizeofPacket;
	
}
//--------------------------HELPER FNS------------------------------------
unsigned char charToHexDigit(char c)
{
	if(islower(c)){
            /* Convert lower case character to upper case 
              using toupper function */
							c = toupper(c);
							} 
  if (c >= 'A')
    return c - 'A' + 10;
  else
    return c - '0';
}

unsigned int StringToHex(char c[2])
{
  return charToHexDigit(c[1]) + 16 * charToHexDigit(c[0]);

}

unsigned int stringToINT(char c[4])
{
  return charToHexDigit(c[3]) + 16 * charToHexDigit(c[2]) + 256 * charToHexDigit(c[1])  + 4096 * charToHexDigit(c[0]);
}

unsigned int StringDecodeAsInteger(char c[4])
{
  return charToHexDigit(c[3]) + charToHexDigit(c[2])*10 + charToHexDigit(c[1]) * 100 + charToHexDigit(c[0])*1000;

}

unsigned long stringToFLOAT(char c[8])
{
  return charToHexDigit(c[7]) + 16 * charToHexDigit(c[6]) + 256 * charToHexDigit(c[5])  + 4096 * charToHexDigit(c[4]) + 65536 * charToHexDigit(c[3]) + 1048576 * charToHexDigit(c[2]) + 16777216 * charToHexDigit(c[1])  +  268435456 * charToHexDigit(c[0]);
}
//----------------------------------------------------------------------------


int hex_msgInfo;
int hex_disable_start_msg;
char state;
char msgInfo[2];

char HMI_BasePacket_Decode(char *receiveBuffer)
{
	char machineID[2];
	char machineType[2];
	char disable_start_msg [2];
	//settings
	char deliverySpeed [4];
	char tensionDraft[8];
	char cylinderSpeed[4];
	char cylinderFeed [8];
	char beaterSpeed [4];
	char beaterFeed[8];
	char conveyorSpeed[8];
	char trunkDelay[4];
	char lengthLimit[4];
	char lengthCorrection[8];
	
	char motorID[4];
	char typeOfTest[4];
	char targetRPM[4];
	char targetSignalVoltage[4];
	char testTime[4];

	//decoded
	char hex_machineID;
 	char hex_machineType;

  unsigned int hex_deliverySpeed ;
	unsigned long hex_tensionDraft ;
	unsigned int hex_cylinderSpeed ;
	unsigned long hex_cylinderFeed ;
	unsigned int	hex_beaterSpeed ;
	unsigned long hex_beaterFeed ;
	unsigned int	hex_conveyorSpeed ;
	unsigned int	hex_trunkDelay ;
	unsigned int	hex_lengthLimit ;
  unsigned int	hex_lengthCorrection ;


	float tensionDraft_F ;
	float cylinderFeed_F;
	float beaterFeed_F;
	float conveyorSpeed_F;
	float lengthCorrection_F;
	
	//diag
	unsigned int	D_motorID ;
	unsigned long D_typeOfTest ;
	unsigned int	D_targetRPM ;
	unsigned int	D_targetSignalVoltage ;
	unsigned int	D_testTime ;
	//start char will be 7E, dont take that out
	//right now we re getting:
	//7E02B0101029900010002007E  - > im paired MSG -> here we want the 99
	//7E020B010102030001010200017E  -> do not start packet -> here we want the 01/02 before the 7E(telling us which screen teh app is in)
	//7E0021010102040001102400DC4020000004DD4093333303203F99999A000C01C200787E -> settings change packet. the identifier is 
	// the 04, and then the vals come.

	strncpy(machineID,receiveBuffer+6,2);
	strncpy(machineType,receiveBuffer+8,2);
	strncpy(msgInfo,receiveBuffer+12,2);
	strncpy(disable_start_msg,receiveBuffer+24,2);
	
	hex_machineID = StringToHex(machineID);
	hex_machineType = StringToHex(machineType);
	hex_msgInfo = StringToHex(msgInfo);
	hex_disable_start_msg = StringToHex(disable_start_msg);
	
	if ((int)hex_msgInfo == FROM_HMI_IM_PAIRED)
		{
		state =1;
		return hex_msgInfo;
		}
	else if ((int)hex_msgInfo == FROM_HMI_DISABLE_MACHINE_START) //siable start can 0,1, or 2
		{
			state = 2;
		return hex_disable_start_msg;
		}
  else if ((int)hex_msgInfo == FROM_HMI_ENABLE_MACHINE_START)  
		{
			state = 3;
		return hex_disable_start_msg;
		}	
	else if ((int)hex_msgInfo == FROM_HMI_CHANGE_PROCESS_PARAMS)
		{
			state = 4;
			// update the update struct
			//7E 00 21 01 01 02 04 00 01 10 24 00DC 40200000 04DD 40933333 0320 3F99999A 000C 01C2 0078 7E 
			strncpy(deliverySpeed,receiveBuffer+22,4);
			strncpy(tensionDraft,receiveBuffer+26,8);
			strncpy(cylinderSpeed,receiveBuffer+34,4);
			strncpy(cylinderFeed,receiveBuffer+38,8);
			strncpy(beaterSpeed,receiveBuffer+46,4);
			strncpy(beaterFeed,receiveBuffer+50,8);
			strncpy(conveyorSpeed,receiveBuffer+58,8);
			strncpy(trunkDelay,receiveBuffer+66,4);
			strncpy(lengthLimit,receiveBuffer+70,4);
		  strncpy(lengthCorrection,receiveBuffer+74,8);
			
			hex_deliverySpeed = stringToINT(deliverySpeed);
			hex_tensionDraft = stringToFLOAT(tensionDraft);
			hex_cylinderSpeed = stringToINT(cylinderSpeed );
			hex_cylinderFeed = stringToFLOAT(cylinderFeed);
			hex_beaterSpeed = stringToINT(beaterSpeed);
			hex_beaterFeed = stringToFLOAT(beaterFeed);
			hex_conveyorSpeed = stringToFLOAT(conveyorSpeed);
			hex_trunkDelay = stringToINT(trunkDelay);
			hex_lengthLimit = stringToINT(lengthLimit);
	    hex_lengthCorrection = stringToFLOAT(lengthCorrection);
	
			tensionDraft_F = *((float*)&hex_tensionDraft);
			cylinderFeed_F = *((float*)&hex_cylinderFeed);
			beaterFeed_F = *((float*)&hex_beaterFeed);
			conveyorSpeed_F = *((float*)&hex_conveyorSpeed);
			lengthCorrection_F =  *((float*)&hex_lengthCorrection);
			
			//update csp directly.
			csp.deliverySpeed = hex_deliverySpeed;
			csp.tensionDraft = tensionDraft_F;
			csp.cylinderSpeed = hex_cylinderSpeed;
			csp.cylinderFeed = cylinderFeed_F;
			csp.beaterSpeed = hex_beaterSpeed;
			csp.beaterFeed = beaterFeed_F;
			csp.conveyorSpeed = conveyorSpeed_F;
			csp.trunkDelay = hex_trunkDelay;
			csp.lengthLimit = hex_lengthLimit;
			csp.lengthCorrection = lengthCorrection_F;
	
			deliverySpeed_perSec = (float)(csp.deliverySpeed)/60.0;
			return FROM_HMI_UPDATED_SETTINGS;
		}
	else if ((int)hex_msgInfo == FROM_HMI_DIAGNOSTIC_TEST)
	{
		state = 5;
		// Update the Diagnostics Struct
		//7E 02 55 00 01 02 05 00 5 01 02 0002 02 02 0005 03 02 0000 04 02 0078 05 02 0025 7E

		strncpy(typeOfTest,receiveBuffer+22,4);
		strncpy(motorID,receiveBuffer+30,4);
		strncpy(targetSignalVoltage,receiveBuffer+38,4);
		strncpy(targetRPM,receiveBuffer+46,4);
		strncpy(testTime,receiveBuffer+54,4);

		D_typeOfTest = stringToINT(typeOfTest);
		D_motorID = StringDecodeAsInteger(motorID);
		D_targetSignalVoltage = stringToINT(targetSignalVoltage);
		D_targetRPM = stringToINT(targetRPM);
		D_testTime = stringToINT(testTime);
		
		D.typeofTest = D_typeOfTest;
		D.motorID = D_motorID;
		D.targetSignal = D_targetSignalVoltage;
		D.targetRPM = D_targetRPM;
		D.testTime = D_testTime;
		
		return FROM_DIAG_UPDATED_TEST_DETAILS;
	}
	else if ((int)hex_msgInfo == FROM_HMI_RESET_LENGTH){
		 state = 5;
		 return hex_msgInfo;
	}
	else
		{
			state =6;
			return NO_VAR;
		}
	
}

