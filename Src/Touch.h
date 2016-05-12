#ifndef __TOUCH_H__
#define __TOUCH_H__

void TOUCH_init(void);

//void PIOINT2_IRQHandler(void);
unsigned int GetTouchADC( unsigned char cmd_code );
unsigned char Read_ADS( unsigned int *x_ad,unsigned int *y_ad );
unsigned char Read_ADS2( unsigned long *x_ad,unsigned long *y_ad );
unsigned char Read_Once( void );
unsigned char Read_Continue( void );
void convert_ad_to_xy( void );


#endif


