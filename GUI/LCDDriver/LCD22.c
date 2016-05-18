/********************************************************************************************************
*
* File                : LCD22.c
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lcd22.h"
//#include "ascii hex(8x16).h"
volatile xy_t touch_xy_buffer[TOUCH_MAX_CACHE];

volatile alt_u8 touch_wr_index;
volatile alt_u8 touch_rd_index;
volatile alt_u8 touch_counter;
alt_u16 color[]= {0xF800,0x07E0,0x001F,0xFFE0,0x0000,0xFFFF,0x07FF,0xF81F};

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

xy_t curr_pot;

#if defined(LCD22_SPI)
void LCD_WR_Data(alt_u16 val)
{
    alt_u8 SPI_DATA;
    LCD_CS_CLR;
    SPI_DATA = (alt_u8)(val>>8);
    //alt_avalon_spi_command(SPI_LCD22_BASE,0,1,&SPI_DATA,0,NULL,ALT_AVALON_SPI_COMMAND_MERGE);
    HAL_SPI_Transmit(&hspi1,&SPI_DATA,1,500);
    SPI_DATA = (alt_u8)val;
    //alt_avalon_spi_command(SPI_LCD22_BASE,0,1,&SPI_DATA,0,NULL,ALT_AVALON_SPI_COMMAND_MERGE);
    HAL_SPI_Transmit(&hspi1,&SPI_DATA,1,500);
}

#elif defined(LCD22_PAN)
void LCD_WR_Data(alt_u16 val)
{
    LCD_CS_CLR;
    LCD_DATA((alt_u8)(val>>8));
    LCD_WR_CLR;
    LCD_WR_SET;
    LCD_DATA((alt_u8)(val));
    LCD_WR_CLR;
    LCD_WR_SET;
}
#endif

void LCD_WR_CMD(alt_u16 index,alt_u16 val)
{
    LCD_RS_CLR;
    LCD_WR_Data(index);
    LCD_RS_SET;
    LCD_WR_Data(val);
}

#pragma optimize = none
alt_u16 get_touch_data(alt_u8 cmd)
{
    alt_u8 tmp1,tmp2;
    alt_u8 SPI_DATA;
    alt_u16 tmp;
    SPI_DATA = cmd;
    //alt_avalon_spi_command(SPI_TOUCH_BASE,0,1,&SPI_DATA,0,NULL,ALT_AVALON_SPI_COMMAND_MERGE);
    HAL_SPI_Transmit(&hspi3,&SPI_DATA,1,500);
    //alt_avalon_spi_command(SPI_TOUCH_BASE,0,0,NULL,1,&tmp1,ALT_AVALON_SPI_COMMAND_MERGE);
    //alt_avalon_spi_command(SPI_TOUCH_BASE,0,0,NULL,1,&tmp2,ALT_AVALON_SPI_COMMAND_MERGE);
    HAL_SPI_Receive(&hspi3,&tmp1,1,500);
    HAL_SPI_Receive(&hspi3,&tmp2,1,500);

    tmp = (((alt_u16)tmp1)<<5) | (((alt_u16)tmp2)>>3);

    return (tmp);
}

xy_t get_touch_xy(void)
{
    xy_t tmp_xy;
	GPIO_PinState ps;

	ps = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	
    if(ps == GPIO_PIN_RESET)
    {
        tmp_xy.x = get_touch_data(TOUCH_CMD_X);
        tmp_xy.y = get_touch_data(TOUCH_CMD_Y);
    }
    else
    {
        tmp_xy.x = 0xFFFF;
        tmp_xy.y = 0xFFFF;
    }
    return tmp_xy;
}

alt_u8 get_point_xy(void)
{
    alt_u8 n,m,tmp;
    xy_t tmp_xy_buf[SAMP_COUNT], tmp_xy;
    alt_u32 tmp_x,tmp_y;

    if(touch_counter>=(TOUCH_MAX_CACHE-1))
    {
        return 0;
    }
    TOUCH_SPI_INIT(&hspi3);
    en_touch();

    for(n=0; n<SAMP_COUNT; n++)
    {
        tmp_xy_buf[n] = get_touch_xy();
    }
    dis_touch();
    for(n=0; n<(SAMP_COUNT-1); n++)
    {
        for(m=0; m<(SAMP_COUNT-n-1); m++)
        {
            tmp = m+1;
            if((tmp_xy_buf[m].x + tmp_xy_buf[m].y) > (tmp_xy_buf[tmp].x + tmp_xy_buf[tmp].y))
            {
                tmp_xy = tmp_xy_buf[tmp];
                tmp_xy_buf[tmp] = tmp_xy_buf[m];
                tmp_xy_buf[m] = tmp_xy;
            }
        }
    }
    if((tmp_xy_buf[SAMP_COUNT/2].x - tmp_xy_buf[SAMP_COUNT/2-1].x > SAMP_THRESHOLD)
            || (tmp_xy_buf[SAMP_COUNT/2].y - tmp_xy_buf[SAMP_COUNT/2-1].y > SAMP_THRESHOLD))
    {
        return 0;
    }

    tmp_x = ((alt_u32)tmp_xy_buf[SAMP_COUNT/2].x + (alt_u32)tmp_xy_buf[SAMP_COUNT/2-1].x)/2;
    tmp_y = ((alt_u32)tmp_xy_buf[SAMP_COUNT/2].y + (alt_u32)tmp_xy_buf[SAMP_COUNT/2-1].y)/2;

    if(tmp_x >= 0xFFF || tmp_y >= 0xFFF)
    {
        return 0;
    }

    touch_xy_buffer[touch_wr_index].x = ((tmp_x * 240)>>12);
    touch_xy_buffer[touch_wr_index].y = ((tmp_y * 320)>>12);

    if(touch_wr_index < (TOUCH_MAX_CACHE-1))
    {
        touch_wr_index++;
    }
    else
    {
        touch_wr_index = 0;
    }

    touch_counter++;

    return 1;
}

alt_u8 get_curr_pot(void)
{
    LCD22_SPI_INIT(&hspi1);
    LCD_CS_CLR;
    if(touch_counter==0)
    {
        return 0;
    }
    touch_counter--;

 	if(touch_rd_index < (TOUCH_MAX_CACHE-1))
    {
        touch_rd_index++;
    }
    else
    {
        touch_rd_index = 0;
    }
	
	curr_pot.x = touch_xy_buffer[touch_rd_index].x;
	curr_pot.y = touch_xy_buffer[touch_rd_index].y;		

	LCD_CS_SET;

	return 1;
}

alt_u8 draw_lcd(void)
{
    alt_u8 n;

    LCD22_SPI_INIT(&hspi1);
    LCD_CS_CLR;
    if(touch_counter==0)
    {
        return 0;
    }
    touch_counter--;
    LCD_WR_CMD(8,touch_xy_buffer[touch_rd_index].x);
    LCD_WR_CMD(10,touch_xy_buffer[touch_rd_index].y);
    LCD_WR_CMD(9,touch_xy_buffer[touch_rd_index].x+(DOT_WIDTH-1));
    LCD_WR_CMD(11,touch_xy_buffer[touch_rd_index].y+(DOT_WIDTH-1));

	curr_pot.x = touch_xy_buffer[touch_rd_index].x;
	curr_pot.y = touch_xy_buffer[touch_rd_index].y;		
	
    if(touch_rd_index < (TOUCH_MAX_CACHE-1))
    {
        touch_rd_index++;
    }
    else
    {
        touch_rd_index = 0;
    }

    LCD_WR_CMD(0x05,0x0010);
    LCD_RS_CLR;
    LCD_WR_Data(0x0E);
    LCD_RS_SET;

    for(n=0; n<(DOT_WIDTH*DOT_WIDTH); n++)
    {
        LCD_WR_Data(COLOR_BLACK);
    }
    LCD_CS_SET;

    return 1;
}

alt_u16* LCD22_panel;

void LCD22_Init()
{
    touch_counter = 0;
    touch_wr_index = 0;
    touch_rd_index = 0;

    //delay_ms(100);
    LCD_CS_SET;
    LCD_CS_CLR;
    lcd_rst();
    LCD22_SPI_INIT(&hspi1);
    LCD_WR_CMD(0x03,0x0001);
    LCD_WR_CMD(0x3A,0x0001);
    //delay_ms(10);
    LCD_WR_CMD(0x24,0x007B);
    LCD_WR_CMD(0x25,0x003B);
    LCD_WR_CMD(0x26,0x0034);
    LCD_WR_CMD(0x27,0x0004);
    LCD_WR_CMD(0x52,0x0025);
    LCD_WR_CMD(0x53,0x0033);
    LCD_WR_CMD(0x61,0x001C);
    LCD_WR_CMD(0x62,0x002C);
    LCD_WR_CMD(0x63,0x0022);
    LCD_WR_CMD(0x64,0x0027);
    LCD_WR_CMD(0x65,0x0014);
    LCD_WR_CMD(0x66,0x0010);
    LCD_WR_CMD(0x2E,0x002D);
    LCD_WR_CMD(0x19,0x0000);
    //delay_ms(20);
    LCD_WR_CMD(0x1A,0x1000);
    LCD_WR_CMD(0x1B,0x0023);
    LCD_WR_CMD(0x1C,0x0C01);
    LCD_WR_CMD(0x1D,0x0000);
    LCD_WR_CMD(0x1E,0x0009);
    LCD_WR_CMD(0x1F,0x0035);
    LCD_WR_CMD(0x20,0x0015);
    LCD_WR_CMD(0x18,0x1E7B);
    LCD_WR_CMD(0x08,0x0000);
    LCD_WR_CMD(0x09,0x00EF);
    LCD_WR_CMD(0x0a,0x0000);
    LCD_WR_CMD(0x0b,0x013F);
    LCD_WR_CMD(0x29,0x0000);
    LCD_WR_CMD(0x2A,0x0000);
    LCD_WR_CMD(0x2B,0x00EF);
    LCD_WR_CMD(0x2C,0x013F);
    LCD_WR_CMD(0x32,0x0002);
    LCD_WR_CMD(0x33,0x0000);
    LCD_WR_CMD(0x37,0x0000);
    LCD_WR_CMD(0x3B,0x0001);
    LCD_WR_CMD(0x04,0x0000);
    LCD_WR_CMD(0x05,0x0010);
    LCD_WR_CMD(0x01,0x0000);
    LCD_WR_CMD(0x00,0x0000);

	LCD22_panel = (alt_u16*)pvPortMalloc(50 * 50 * 2);

	int i;
	for(i=0;i<50 * 50 * 2;i++)
	{
		LCD22_panel[i] = 0xFF0000;//GUI_BLUE
	}
}

void LCD_test()
{
    alt_u16 temp,num,i;
    alt_u8 n;

    LCD_WR_CMD(0x08,0x0000);
    LCD_WR_CMD(0x0a,0x0000);
    LCD_WR_CMD(0x09,0x00EF);
    LCD_WR_CMD(0x0b,0x013F);

    LCD_WR_CMD(0x06,0x0000);
    LCD_WR_CMD(0x07,0x0000);

    LCD_RS_CLR;
    LCD_WR_Data(0x0E);
    LCD_RS_SET;

    for(n=0; n<8; n++)
    {
        temp = color[n];
        for(num=40*240; num>0; num--)
        {
            LCD_WR_Data(temp);
        }
        delay_ms(500);
    }
    for(n=0; n<8; n++)
    {
        LCD_WR_CMD(0x08,0x0000);
        LCD_WR_CMD(0x0a,0x0000);
        LCD_WR_CMD(0x09,0x00EF);
        LCD_WR_CMD(0x0b,0x013F);

        LCD_WR_CMD(0x06,0x0000);
        LCD_WR_CMD(0x07,0x0000);

        LCD_RS_CLR;
        LCD_WR_Data(0x0E);
        LCD_RS_SET;
        temp = color[n];
        for(i=0; i<240; i++)
        {
            for(num=0; num<320; num++)
            {
                LCD_WR_Data(temp);
            }
        }
        delay_ms(500);
    }
    LCD_CS_SET;
}

void LCD_clear(alt_u16 p)
{
    alt_u16 i,j;
    //LCD_WR_CMD(0x08,0x0000);
    //LCD_WR_CMD(0x0a,0x0000);
    //LCD_WR_CMD(0x09,0x00EF);
    //LCD_WR_CMD(0x0b,0x013F);

    //LCD_WR_CMD(0x06,0x0000);
    //LCD_WR_CMD(0x07,0x0000);

    LCD_RS_CLR;
    LCD_WR_Data(0x0E);
    LCD_RS_SET;

    for(i=0; i<320; i++)
    {
        for(j=0; j<240; j++)
        {
            LCD_WR_Data(p);
        }
    }
    LCD_CS_SET;
}

void write_dot(alt_u8 dx,alt_u16 dy,alt_u16 index)
{
	LCD_WR_CMD(0x08,0x00);
	LCD_WR_CMD(0x0a,0x0000);
	LCD_WR_CMD(0x09,0xEF);
	LCD_WR_CMD(0x0b,0x013F);

	LCD_WR_CMD(0x06,dx);
	LCD_WR_CMD(0x07,dy);
	LCD_WR_CMD(0x0E,index);
}
/*void DisplayChar(alt_u8 casc,alt_u8 postion_x,alt_u8 postion_y)
{
	alt_u8 i,j,b;
	alt_u8 *p;

	LCD_WR_CMD(0x08,postion_x*8); 		//x start point
	LCD_WR_CMD(0x0a,postion_y*16); 		//y start point
	LCD_WR_CMD(0x09,postion_x*8+7);		//x end point
	LCD_WR_CMD(0x0b,postion_y*16+15);	//y end point

	LCD_WR_CMD(0x06,postion_x*8);
	LCD_WR_CMD(0x07,postion_y*16);

	LCD_RS_CLR;
	LCD_WR_Data(0x0E);
	LCD_RS_SET;
	p = (alt_u8*)ascii;
	p += casc*16;
	for(j=0;j<16;j++)
	{
		b = *(p+j);
		for(i=0;i<8;i++)
		{
			if(b&0x80)
			{
				LCD_WR_Data(0xffff);
			}
			else
			{
				LCD_WR_Data(0x0000);
			}
			b <<= 1;
		}
	}
	LCD_CS_SET;
}

void DisplayString(alt_u8 *s,alt_u8 x,alt_u8 y)//Ó¢ÎÄ×Ö·û´®ÏÔÊ¾.
{
	while (*s)
	{
		DisplayChar(*s,x,y);
		if(++x>=30)
		{
			x = 0;
			if(++y>=20)
			{
				y = 0;
			}
		}
		s++;
    }
}
*/