#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Serial.h"

uint8_t RxData;

int main(void)
{

	Serial_Init();
	
	while (1)
	{
		Serial_SendString("Hello world!\r\n");
		Delay_ms(500);
	}
}
