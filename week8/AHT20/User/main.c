#include "delay.h"
#include "usart.h"
#include "bsp_i2c.h"


int main(void)
{	
	delay_init();  	  
	uart_init(115200);	
	IIC_Init();
	while(1)
	{
		printf("Please wait a moment:");
		read_AHT20_once();
		delay_ms(1500);
	}
}
