#ifndef __HMIFNS_H
#define __HMIFNS_H

#include "HMI_Constants.h"
#include "Structs.h"


struct HMI_InfoBase
{
    char start;
    char sender;
    char packetLength;
    char machineType;
    char machineID;
    char msgType;
    char nextscreen;
    char screen_substate;
    char attributeCount;
    char endChar;
};


struct HMI_RunPacket
{
    char tlv1Code;
    char tlv1length;
    int tlv1_val_int;
    float tlv1_val_float;
    char tlv2Code;
    char tlv2length;
    int tlv2_val;
    char tlv3Code;
    char tlv3length;
    int tlv3_val;
};

struct HMI_DiagPacket
{
    char tlv1Code;
    char tlv1length;
    int tlv1_val;
    char tlv2Code;
    char tlv2length;
    int tlv2_val;
    char tlv3Code;
    char tlv3length;
    int tlv3_val;
		char tlv4Code;
    char tlv4length;
    int tlv4_val;
};

struct HMI_StopPacket
{
    char tlv1Code;
    char tlv1length;
    int tlv1_val;
    char tlv2Code;
    char tlv2length;
    int tlv2_val;
    char tlv3Code;
    char tlv3length;
    int tlv3_val;
};



void Create_HMI_BasePacket(void);
void Create_HMI_Run_Packet(void);
void Create_HMI_StopPacket(void);
void Create_HMI_DiagPacket(void);

void UpdateBasePacket_Modes(char machineType,char msgType,char state,char runsubMode,char packetLength,char attributeCount);
void UpdateRunPacket(char tlv1Type,char tlv1Code,int tlv1Val_i,float tlv1Val_f,char tlv2Code,int tlv2Val,char tlv3Code,int tlv3Val);
void UpdateStopPacket(char tlv1Code,char tlv2Code,char tlv3Code);


int UpdateRunPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_RunPacket hrp,int cyl_RPM,int btr_RPM,int production);
int UpdateStopPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,float error_val); // Continuously
int UpdatePausePacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_StopPacket hsp,int stop_reason,int motor_fault,float error_val); //differnet fn bcos 00 needs to be sent as 00000000
int UpdateDiagPacketString(char *buffer,struct HMI_InfoBase hsb,struct HMI_DiagPacket hdp,int typeOfTest,int motorID,int signalVoltage,int rpm);
int HMI_GetCardingMachineSettingsAllPacketString(char *buffer,struct HMI_InfoBase hsb,struct CardingSettings csp);
int HMI_GetIdlePacketString(char *buffer,struct HMI_InfoBase hsb);
int HMI_GetSettingsACKPacketString(char *buffer,struct HMI_InfoBase hsb);
int HMI_Get_DiagOver_PacketString(char *buffer,struct HMI_InfoBase hsb,char status);
char HMI_BasePacket_Decode(char *receiveBuffer);

//helper fns
unsigned long stringToFLOAT(char c[8]);
unsigned int stringToINT(char c[4]);
unsigned char charToHexDigit(char c);
unsigned int StringToHex(char c[2]);
unsigned int StringDecodeAsInteger(char c[4]);


extern struct HMI_InfoBase hsb;
extern struct HMI_RunPacket hrp;
extern struct HMI_StopPacket hsp;
extern struct HMI_DiagPacket hdp;
#endif

