
#include "include.h"

void setup() 
{
	pinMode(LED,OUTPUT);
	pinMode(BUZZER,OUTPUT);
 
	InitPWM();
	InitTimer2();
	
	InitUart1();
	InitPS2();
	InitFlash();
	InitMemory();
}


void loop() 
{
	TaskRun(); 
}




