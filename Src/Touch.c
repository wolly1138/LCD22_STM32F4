#include "stm32f4xx_hal.h"

#include "LTM022A69B.h"
#include "LCD_lib.h"

#include"touch.h"

//#define TOUCH_nCS_H()    PORTB |= (1<<PB4);
//#define TOUCH_nCS_L()    PORTB &= ~(1<<PB4);

#define TOUCH_nCS_H()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET)
#define TOUCH_nCS_L()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET)

#define CMD_READ_X    0xD0  //0B11010000���ò�ַ�ʽ��X����
#define CMD_READ_Y    0x90  //0B10010000���ò�ַ�ʽ��Y����

 typedef struct 
{
	unsigned int  x;//LCD����
	unsigned int  y;
	unsigned long x_ad_val; //ADCֵ
	unsigned long y_ad_val;						   	    
	unsigned char  pen_status;//�ʵ�״̬			  
}_touch_dot;


_touch_dot touch_dot,*p_touch_dot;//�����



void TOUCH_init(void)
{
//	DDRB |=0X10;
//	PORTB |=0X10;
}


void TOUCH_WRITE_REG(unsigned int index)
{
	
	LCD_RS_L();
	TOUCH_nCS_L();

    SPI0_communication((unsigned char)(index>>8));    //00000000 000000000
    SPI0_communication((unsigned char)(index));

	TOUCH_nCS_H();
	LCD_RS_H();

}

/******************************************************************************
* Function Name  : LCD_WRITE_COMMAND
* Description    : send command to LCD
* Input          : index, data
* Output         : None
* Return         : None
******************************************************************************/
void TOUCH_WRITE_COMMAND(unsigned int index,unsigned int data)
{
	//select command register

	LCD_RS_L();
	TOUCH_nCS_L();

    SPI0_communication((unsigned char)(index>>8));    //00000000 000000000
    SPI0_communication((unsigned char)(index));
	LCD_CS_H();
	//send data
	LCD_RS_H();

	TOUCH_nCS_L();
    SPI0_communication((unsigned char)(data>>8));    //00000000 000000000
    SPI0_communication((unsigned char)(data));
	TOUCH_nCS_H();

}

/*******************************************************************************
* Function Name  : LCD_WRITE_DATA
* Description    : write data to LCD
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TOUCH_WRITE_DATA(unsigned int data)
{
    SPI0_communication((unsigned char)(data>>8));    //00000000 000000000
    SPI0_communication((unsigned char)(data));
}

/******************************************************************************
* Function Name  : GetTouchADC
* Description    : ��ȡ1��XPT2046����ȡ1��ADC�����
* Input          : ����XPT2046�������ֽ�.
                    ֻ�������µĴ���:
                    CMD_READ_X:��ȡX��ADCֵ                
                    CMD_READ_Y:��ȡY��ADCֵ
* Output         : None
* Return         : ADת���Ľ��.
******************************************************************************/
unsigned int GetTouchADC (unsigned char data)
{	  
	unsigned int NUMH , NUML;
	unsigned int Num;
	LCD_CS_H();
	TOUCH_nCS_L();
		
	HAL_Delay(2);
	SPI0_communication(data);
	HAL_Delay(2);              // ��ʱ�ȴ�ת�����
	NUMH=SPI0_communication(0x00);
	NUML=SPI0_communication(0x00);
	Num=((NUMH)<<8)+NUML; 	
	Num>>=4;                // ֻ�и�12λ��Ч.
	TOUCH_nCS_H();

	return(Num);   
}


#define READ_TIMES 10 //��ȡ����
#define LOST_VAL 4	  //����ֵ

/*************************************************
 ��    �ܣ�  ��ȡ����XPT2046,��ȡADC���,������ READ_TIMES ����.
             ����һ��������ȣ��������Է���ֵ������ɸѡ.
 ��ڲ�����cmd_code:����XPT2046�Ŀ����ֽ�.
            ֻ�������µĴ���:
            CMD_READ_X:��ȡX��ADCֵ                
            CMD_READ_Y:��ȡY��ADCֵ
*************************************************/

unsigned int GetTouchADCEx(unsigned char cmd_code)
{
	unsigned int i, j;
	unsigned int buf[READ_TIMES];
	unsigned int sum=0;
	unsigned int temp;

	for(i=0;i<READ_TIMES;i++)
	{				 
		buf[i]=GetTouchADC(cmd_code);	    
	}				    
	for(i=0;i<READ_TIMES-1; i++)//����
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//��������
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
}


/*************************************************/
/* ��    �ܣ���ȡX������ADCֵ                  */
/* ��ڲ�����&touch_dot.x_ad_val,&touch_dot.y_ad_val   */
/* ���ڲ�����0���ɹ������ص�X,Y_ADCֵ��Ч��      */
/*           1: ʧ�ܣ����ص�X,Y_ADCֵ��Ч��      */ 
/*************************************************/
unsigned char Read_ADS(unsigned int *x_ad,unsigned int *y_ad)
{
	unsigned int xtemp,ytemp;			 	 		  
	xtemp=GetTouchADCEx(CMD_READ_X);    //��ɸѡ�Ķ�ȡX��ADת�����
	ytemp=GetTouchADCEx(CMD_READ_Y);	    //��ɸѡ�Ķ�ȡY��ADת�����											   
	if(xtemp<100||ytemp<100)
        return 1;   //����ʧ��
	*x_ad=xtemp;
	*y_ad=ytemp;        
	return 0;//�����ɹ�
}

/*************************************************/
/* ���ܣ��������ζ�ȡADCֵ						 */
/* ԭ�������ζ�ȡ��ֵ���Ƚϣ�����Χ�ڿ�ȡ  */ 
/* ��ڲ�����x:&touch_dot.x_ad_val(X����ADCֵ)      */
/*           y:&touch_dot.y_ad_val(Y����ADCֵ)      */
/* ���ڲ�����0���ɹ������ص�X,Y_ADCֵ��Ч��      */
/*           1: ʧ�ܣ����ص�X,Y_ADCֵ��Ч��      */ 
/*************************************************/
#define ERR_RANGE 50 //��Χ 

unsigned char Read_ADS2(unsigned long *x_ad,unsigned long *y_ad) 
{
	unsigned int x1,y1;
 	unsigned int x2,y2;
 	unsigned char res; 

    res=Read_ADS(&x1,&y1);  // ��һ�ζ�ȡADCֵ 
    if(res==1)  return(1);	// �������ʧ�ܣ�����1
    res=Read_ADS(&x2,&y2);	// �ڶ��ζ�ȡADCֵ   
    if(res==1)  return(1);   	// �������ʧ�ܣ�����1
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
        &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x_ad=(x1+x2)/2;
        *y_ad=(y1+y2)/2;
        return 0;	 // ��ȷ��ȡ������0
    }
    else return 1;	 // ǰ����+-50�ڣ��������� 
} 

/*************************************************/
/* ���ܣ��Ѷ�����ADCֵת��������ֵ               */
/*************************************************/	  
void convert_ad_to_xy(void)
{
//	touch_dot.x=(240-(touch_dot.x_ad_val-100)/7.500); // �Ѷ�����X_ADCֵת����TFT X����ֵ
//	touch_dot.y=(320-(touch_dot.y_ad_val-135)/5.705); // �Ѷ�����Y_ADCֵת����TFT Y����ֵ
//    touch_dot.x = (((touch_dot.x_ad_val * 240)>>12)-110)*2;
//    touch_dot.y = (((touch_dot.y_ad_val * 320)>>12)-150)*2;

//X=(240 * AD - 2100* 240) / 1900
//Y=(320 * AD - 2100* 320) / 1900
    touch_dot.x=(240*touch_dot.x_ad_val -2100*240)/ 1900;
    touch_dot.y=(320*touch_dot.y_ad_val -2100*320)/ 1900;

}
 
/*************************************************/
/* ���ܣ���ȡһ��XY����ֵ                        */
/*************************************************/	
unsigned char Read_Once(void)
{
//	touch_dot.pen_status=Pen_Up;
	if(Read_ADS2(&touch_dot.x_ad_val,&touch_dot.y_ad_val)==0)	// �����ȡ���ݳɹ�
	{
		//while((PINE & (1<<PE4))==0);   // �����ǲ��ǻ�������:IRQΪ�͵�ƽ(bit7Ϊ0)˵����������
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET);
		convert_ad_to_xy();   // �Ѷ�����ADCֵת���TFT����ֵ
		return 0;	// ����0����ʾ�ɹ�
	}
	else return 1;	// �����ȡ����ʧ�ܣ�����1��ʾʧ��
}
/*************************************************/
/* ���ܣ�������ȡXY����ֵ                        */
/*************************************************/
unsigned char Read_Continue(void)
{
//	touch_dot.pen_status=Pen_Up;	  
	if(Read_ADS2( &touch_dot.x_ad_val,&touch_dot.y_ad_val )==0)	 // �����ȡ���ݳɹ�
	{
		convert_ad_to_xy();   // �Ѷ�����ADCֵת���TFT����ֵ
		return 0;	   // ����0����ʾ�ɹ�
	}
	else return 1;	   // �����ȡ����ʧ�ܣ�����1��ʾʧ��
}










