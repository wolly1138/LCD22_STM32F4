/********************************************************************************************************
*
* File                : LCD22.h
* Hardware Environment:
* Build Environment   : Quartus II Version 10.1 / Nios II 10.1
* Version             :
* By                  : Su Wei Feng
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

#ifndef LCD22_H_
#define LCD22_H_

#define delay_ms(ms)	osDelay(ms)//vTaskDelay(ms/portTICK_RATE_MS)

#define alt_u32			unsigned int
#define alt_u16			unsigned short
#define alt_u8			unsigned char

#define LCD22_SPI
//#define LCD22_PAN

#define LCD_CS_CLR      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET)
#define LCD_CS_SET     	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET)
#define en_touch()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET)
#define dis_touch()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET)
#define LCD_RS_SET    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET)
#define LCD_RS_CLR   	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET)
#define LCD_RST_CLR     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_RESET)
#define LCD_RST_SET     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET)
#define PENIRQ          HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define busy_touch      HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)

#ifdef LCD22_PAN
#define LCD_DATA(data)  IOWR_ALTERA_AVALON_PIO_DATA(PD_BASE, data)
#define LCD_WR_CLR      IOWR_ALTERA_AVALON_PIO_DATA(LCD_WR_BASE, 0)
#define LCD_WR_SET      IOWR_ALTERA_AVALON_PIO_DATA(LCD_WR_BASE, 1)
#define lcd_wr_clk()   {LCD_WR_CLR;LCD_WR_SET;}
#endif

#define lcd_rst()      {LCD_RST_CLR;delay_ms(1);LCD_RST_SET;delay_ms(1);}
#define LCD22_SPI_INIT(hspi)  HAL_SPI_Init(hspi)
#define TOUCH_SPI_INIT(hspi)  HAL_SPI_Init(hspi)

#define COLOR_YELLOW 0xFFE0
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_INIT 0x00FF

#define SAMP_COUNT 4
#define SAMP_THRESHOLD 5

#define TOUCH_CMD_X 0xD0
#define TOUCH_CMD_Y 0x90

#define DOT_WIDTH 2
#define TOUCH_MAX_CACHE 8

void LCD_Init();
void LCD_WR_CMD(alt_u16 index,alt_u16 val);
void LCD_WR_Data(alt_u16 val);
void LCD_test();
void LCD_clear(alt_u16 p);

void DisplayChar(alt_u8 casc,alt_u8 postion_x,alt_u8 postion_y);
void DisplayString(alt_u8 *s,alt_u8 x,alt_u8 y);	//Ó¢ÎÄ×Ö·û´®ÏÔÊ¾.

typedef struct xy
{
	alt_u16 x;
	alt_u16 y;
}xy_t;

alt_u16 get_touch_data(alt_u8 cmd);
xy_t get_touch_xy(void);
alt_u8 get_point_xy(void);
alt_u8 draw_lcd(void);

//volatile xy_t touch_xy_buffer[TOUCH_MAX_CACHE];

#endif /* LCD22_H_ */
