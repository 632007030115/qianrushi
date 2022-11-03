#include "sys.h"


//STM32F103ºËÐÄ°åÀý³Ì
//¿âº¯Êý°æ±¾Àý³Ì
/********** mcudev.taobao.com ³öÆ·  ********/

//	 

//STM32¿ª·¢°å
//ÏµÍ³ÖÐ¶Ï·Ö×éÉèÖÃ»¯		   

//********************************************************************************  
void NVIC_Configuration(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//ÉèÖÃNVICÖÐ¶Ï·Ö×é2:2Î»ÇÀÕ¼ÓÅÏÈ¼¶£¬2Î»ÏìÓ¦ÓÅÏÈ¼¶

}

