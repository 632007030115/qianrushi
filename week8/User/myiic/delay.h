#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"
//	 

//STM32F103ºËÐÄ°åÀý³Ì
//¿âº¯Êý°æ±¾Àý³Ì
/********** mcudev.taobao.com ³öÆ·  ********/


//Ê¹ÓÃSysTickµÄÆÕÍ¨¼ÆÊýÄ£Ê½¶ÔÑÓ³Ù½øÐÐ¹ÜÀí
//°üÀ¨delay_us,delay_ms

// 	 
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif
