
#include "encoder.h"
#include "main.h"
#include "logicDefines.h"
#include "Initialize.h"
#include "functionDefines.h"

int Cap1 = 0;
int Cap2 = 0;
int Cap3 = 0;
int Cap4 = 0;
int Cap5 = 0;
int Cap6 = 0;
int Cap7 = 0;
int Cap8 = 0;
int Cap9 = 0;
int Cap10 = 0;

int Rpm1 = 0;
int Rpm2 = 0;
int Rpm3 = 0;
int Rpm4 = 0;
int Rpm5 = 0;
int Rpm6 = 0;
int Rpm7 = 0;
int Rpm8 = 0;
int Rpm9 = 0;
int Rpm10 = 0;

int filter1 = 0;
int filter2 = 0;
int filter3 = 0;
int filter4 = 0;
int filter5 = 0;
int filter6 = 0;
int filter7 = 0;
int filter8 = 0;
int filter9 = 0;
int filter10 = 0;



void ResetSecondaryEncoderVariables(void)
{
	
		Cap3 = 0;
		Cap4 = 0;
		Cap5 = 0;
		Cap6 = 0;
		Cap7 = 0;
		Cap8 = 0;
		Cap9 = 0;
		Cap10 = 0;

		Rpm3 = 0;
		Rpm4 = 0;
		Rpm5 = 0;
		Rpm6 = 0;
		Rpm7 = 0;
		Rpm8 = 0;
		Rpm9 = 0;
		Rpm10 = 0;

		filter3 = 0;
		filter4 = 0;
		filter5 = 0;
		filter6 = 0;
		filter7 = 0;
		filter8 = 0;
		filter9 = 0;
		filter10 = 0;
}

void ResetEncoderVariables(void)
{
		Cap1 = 0;
		Cap2 = 0;
		Cap3 = 0;
		Cap4 = 0;
		Cap5 = 0;
		Cap6 = 0;
		Cap7 = 0;
		Cap8 = 0;
		Cap9 = 0;
		Cap10 = 0;

		Rpm1 = 0;
		Rpm2 = 0;
		Rpm3 = 0;
		Rpm4 = 0;
		Rpm5 = 0;
		Rpm6 = 0;
		Rpm7 = 0;
		Rpm8 = 0;
		Rpm9 = 0;
		Rpm10 = 0;

		filter1 = 0;
		filter2 = 0;
		filter3 = 0;
		filter4 = 0;
		filter5 = 0;
		filter6 = 0;
		filter7 = 0;
		filter8 = 0;
		filter9 = 0;
		filter10 = 0;
}

uint16_t Cap1_F = 0;
uint16_t Cap2_F = 0;
uint16_t Cap3_F = 0;
uint16_t Cap4_F = 0;
uint16_t Cap5_F = 0;
uint16_t Cap6_F = 0;
uint16_t Cap7_F = 0;

void UpdateRpm(void)
{	
		Rpm1 = (Cap1*10.0*60.0)/16.0;
		Cap1_F = Cap1;
		filter1 = Rpm1;//((filter1*0.70)+(Rpm1*0.30));
		Cap1 = 0;
		
		Rpm2 = (Cap2*10.0*60.0)/16.0;
		Cap2_F = Cap2;
		filter2 = Rpm2;//((filter2*0.70)+(Rpm2*0.30));
		Cap2 = 0;
	

		Rpm3 = (Cap3*10.0*60.0)/8.0;
		Cap3_F = Cap3;
		filter3 = Rpm3;//((filter3*0.70)+(Rpm3*0.30));
		Cap3 = 0;
	
		Rpm4 = (Cap4*10.0*60.0)/8.0;
		Cap4_F = Cap4;
		filter4 = Rpm4;//((filter4*0.70)+(Rpm4*0.30));
		Cap4 = 0;
	
		Rpm5 = (Cap5*10.0*60.0)/8.0;
		Cap5_F = Cap5;
		filter5= Rpm5;//((filter5*0.70)+(Rpm5*0.30));
		Cap5 = 0;
	
		Rpm6 = (Cap6*10.0*60.0)/8.0;
		Cap6_F = Cap6;
		filter6 = Rpm6;//((filter6*0.70)+(Rpm6*0.30));
		Cap6 = 0;
		
		Rpm7 = (Cap7*10.0*60.0)/8.0;
		Cap7_F = Cap7;
		filter7 = Rpm7;//((filter7*0.70)+(Rpm7*0.30));
		Cap7 = 0;
		
	//Rpm8 = (Cap8*10.0*60.0)/8.0;
	//filter8 = ((filter8*0.70)+(Rpm8*0.30));
	//Cap8 = 0;
		
}

int FilterRpm(char motorIndex)
{
	int rpm;
	if (motorIndex == MOTOR1)
	{
		rpm = filter1;
	}
	if (motorIndex == MOTOR2)
	{
		rpm = filter2;
	}
	if (motorIndex == MOTOR3)
	{
		rpm = filter3;
	}
	if (motorIndex == MOTOR4)
	{
		rpm = filter4;
	}
	if (motorIndex == MOTOR5)
	{
		rpm = filter5;
	}
	if (motorIndex == MOTOR6)
	{
		rpm = filter6;
	}
	if (motorIndex == MOTOR7)
	{
		rpm = filter7;
	}
	if (motorIndex == MOTOR8)
	{
		rpm = filter8;
	}
	return rpm;
}
























