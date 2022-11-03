#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "sys.h"
#include "delay.h"
#include "usart.h"
//Ê¹ÓÃIIC1 ¹ÒÔØM24C02,OLED,LM75AD,HT1382    PB6,PB7
 
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}
 
//IO²Ù×÷º¯Êý	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //ÊäÈëSDA 


//IICËùÓÐ²Ù×÷º¯Êý
void IIC_Init(void);                //³õÊ¼»¯IICµÄIO¿Ú				 
void IIC_Start(void);				//·¢ËÍIIC¿ªÊ¼ÐÅºÅ
void IIC_Stop(void);	  			//·¢ËÍIICÍ£Ö¹ÐÅºÅ
void IIC_Send_Byte(u8 txd);			//IIC·¢ËÍÒ»¸ö×Ö½Ú
u8 IIC_Read_Byte(unsigned char ack);//IIC¶ÁÈ¡Ò»¸ö×Ö½Ú
u8 IIC_Wait_Ack(void); 				//IICµÈ´ýACKÐÅºÅ
void IIC_Ack(void);					//IIC·¢ËÍACKÐÅºÅ
void IIC_NAck(void);				//IIC²»·¢ËÍACKÐÅºÅ
 
void IIC_WriteByte(uint16_t addr,uint8_t data,uint8_t device_addr);
uint16_t IIC_ReadByte(uint16_t addr,uint8_t device_addr,uint8_t ByteNumToRead);//¼Ä´æÆ÷µØÖ·£¬Æ÷¼þµØÖ·£¬Òª¶ÁµÄ×Ö½ÚÊý  


void  read_AHT20_once(void);
void  reset_AHT20(void);
void  init_AHT20(void);	
void  startMeasure_AHT20(void);
void  read_AHT20(void);
uint8_t  Receive_ACK(void);
void  Send_ACK(void);
void  SendNot_Ack(void);
void I2C_WriteByte(uint8_t  input);
uint8_t I2C_ReadByte(void);	
void  set_AHT20sendOutData(void);
void  I2C_Start(void);
void  I2C_Stop(void);
#endif

