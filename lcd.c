/*
 * lcd.c
 *
 *  Created on: 2017. 2. 15.
 *      Author: ksyoo
 */
#include  <msp430g2433.h>

#include <stdlib.h>
#include <stdint.h>

#include "font.h"
#include "lcd.h"
#include "io.h"

extern uint8_t lcd_data[][20];
extern	uint8_t	user_font[8][5];
extern uint8_t cursor_sts, cursor_x, cursor_y;
extern uint16_t timerA_count ;

uint8_t lcd_buf[120];
uint8_t	blink_time = 0, blink_flag = 0;


void LCD_light_onoff(uint8_t mode)
{
	if ( !mode ) {
		//Uart_TxStr("Loff");
		LCD_LIGHT_OUT |= LCD_LIGHT;
	}
	else {
		//Uart_TxStr("Lon");
		LCD_LIGHT_OUT &= ~LCD_LIGHT;
	}
}

void LCD_clear_screen(void) //Clear screen
{
	//printf("%s\n", __FUNCTION__);
	LCD_command(0x01); //Clear screen display
	LCD_command(0x34); // Move the cursor display settings
	LCD_command(0x30); // The cursor displays the power setting
}

void LCD_init()
{
	LCD_CONTROL_DIR = LCD_CS | LCD_A0 | LCD_RD | LCD_WR;
	LCD_LIGHT_DIR |= LCD_LIGHT;

	lcd_control(LCD_BIAS_0); 		//
	lcd_control(LCD_ADC_NORMAL);  	//
	lcd_control(0xc8);	// LCD_COM_NORMAL = 0xc0

	lcd_control(LCD_BOOST_MODE);		// booster ratio select mode set
	lcd_control(LCD_BOOST_RATIO_0);	// use 4x

    lcd_control(0x27);				// V5 voltage control

	lcd_control(0x81);				// Electronic volume set
	lcd_control(0xd);				//  3f Electronic volume control

	lcd_control(LCD_POWER_CONTROL | 0x07);
	lcd_control(LCD_DISP_NORMAL); 	//Display mode settings,
	lcd_control(LCD_ALL_NORMAL);		//
	lcd_control(LCD_DISPLAY_ON);		//
	LCD_light_onoff(1);

	LCD_clear_screen();
	LCD_clear();
}

#if 0
void LCD_write_control(uint8_t cmd)
{
	LCD_BUSY();

	LCD_CONTROL_ON;
	LCD_BUSY();
	LCD_WRITE_BUS(cmd);
	LCD_CONTROL_OFF;
}


void LCD_write_control_data(uint8_t cmd, uint8_t data)
{
	LCD_BUSY();

	LCD_CONTROL_ON;
	LCD_WRITE_BUS(cmd);
	LCD_WRITE_BUS(data);

	LCD_CONTROL_OFF;
}

void LCD_write_data(uint8_t data)
{
	LCD_BUSY();

	LCD_DATA_ON;
	LCD_WRITE_BUS(data);
	LCD_DATA_OFF;
}


void LCD_write_buf(uint8_t *data, uint8_t size)
{
	uint8_t p;

	LCD_BUSY();

	LCD_DATA_ON;
	// check busy flag
	p = *data;
	while (size--) {
		LCD_WRITE_BUS(p);
		data++;
	}

	LCD_DATA_OFF;
}


uint8_t LCD_read_status()
{
	uint8_t d0, d6;

	LCD_BUSY();

	LCD_CONTROL_ON;
	LCD_READ_BUS();

	LCD_CONTROL &= ~LCD_RD;
	NOP();
	NOP();
	d0 = LCD_D0_IN & 0x3f;
	d6 = LCD_D6_IN;

	d0 = (d6 & 0x80 ) | ((d6 << 1) & 0x40);

	LCD_CONTROL |= LCD_RD;
	LCD_CONTROL_OFF;

	return d0;

}

uint8_t LCD_read_data()
{
	uint8_t d0, d6;

	LCD_BUSY();

	LCD_DATA_ON;
	LCD_READ_BUS();

	LCD_CONTROL &= ~LCD_RD;
	NOP();
	NOP();
	d0 = LCD_D0_IN & 0x3f;
	d6 = LCD_D6_IN;

	d0 = (d6 & 0x80 ) | ((d6 << 1) & 0x40);

	LCD_CONTROL |= LCD_RD;
	LCD_DATA_OFF;

	return d0;
}

void LCD_display(uint8_t mode)
{
	uint8_t cmd;
	cmd = LCD_DISPLAY;
	if ( mode ) cmd |= 0x01;

	LCD_write_control(cmd);
}


void LCD_start_line(uint8_t addr)
{
	uint8_t cmd;

	cmd = LCD_START_LINE | (addr & 0x3f);
	LCD_write_control(cmd);
}

void LCD_set_page(uint8_t addr)
{
	uint8_t cmd;

	cmd = LCD_SET_PAGE | (addr & 0x0f);
	LCD_write_control(cmd);
}

void LCD_set_upper(uint8_t addr)
{
	uint8_t cmd;

	cmd = LCD_SET_UPPER | (addr & 0x0f);
	LCD_write_control(cmd);
}

void LCD_set_lower(uint8_t addr)
{
	uint8_t cmd;

	cmd = LCD_SET_LOWER | (addr & 0x0f);
	LCD_write_control(cmd);
}


void LCD_sel_adc(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_SEL_ADC | (mode & 0x01);
	LCD_write_control(cmd);
}

void LCD_disp_normal(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_DISP_NORMAL | (mode & 0x01);
	LCD_write_control(cmd);
}

void LCD_disp_all(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_DISP_ALL | (mode & 0x01);
	LCD_write_control(cmd);
}

void LCD_set_bias(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_SET_BIAS | (mode & 0x01);
	LCD_write_control(cmd);
}

void LCD_inc_addr(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_INC_ADDR;
	if ( mode == 0 ) cmd |= 0x0E;

	LCD_write_control(cmd);
}

void LCD_reset()
{
	LCD_write_control(LCD_RESET);
}

void LCD_set_dir(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_SET_DIR;
	if ( mode == 1 ) cmd |= 0x08;
	LCD_write_control(cmd);
}

void LCD_set_power(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_SET_POWER | (mode & 0x0f);
	LCD_write_control(cmd);
}


void LCD_set_volume(uint8_t mode)
{
	uint8_t cmd, data;

	cmd = LCD_SET_VOLUME;
	data = mode & 0x3f;
	LCD_write_control_data(cmd, data);
}


void LCD_set_sleep(uint8_t mode)
{
	uint8_t cmd;

	cmd = LCD_SET_SLEEP | (mode & 0x01);
	LCD_write_control_data(cmd, 0);
}

void LCD_set_booster(uint8_t value)
{
	uint8_t cmd,data;

	cmd = LCD_SET_BOOSTER;
	data = value & 0x03;
	LCD_write_control_data(cmd, 0);
}

void LCD_nop()
{
	LCD_write_control(LCD_NOP);
}

#if 0		// don't use test command
void LCD_test()
{
	LCD_write_control(LCD_TEST);
}
#endif
#endif

void LCD_clear()
{
	int8_t i;
	uint8_t x;

	lcd_control(LCD_START_LINE_0);
	for (i = 7; i >= 0; i--)
	{
		lcd_control(LCD_PAGE_ADDR_0 + i); //page address
		lcd_control(LCD_COL_ADDR_H_0); // col. addr high
		lcd_control(LCD_COL_ADDR_L_0); // col. addr low
		for(x=0;x<128;x++)
		{
			lcd_data(0x00); //Read data written to the LCD
		}
	}
}

void LCD_row_bitmap(uint8_t row)
{
	uint8_t *code, *buf, *code1;
	uint8_t const  *font;
	uint8_t i,j, line, m_offset, p_offset, no_font;

	switch (row) {
		case 1:
			line = 0;
			p_offset = 0;
			m_offset = 4;
			break;
		case 2:
			line = 0;
			p_offset = 4;
			m_offset = 2;
			break;
		case 3:
			line = 1;
			p_offset = 6;
			m_offset = 0;
			break;
		case 4:
			line = 2;
			p_offset = 8;
			m_offset = 0;
			break;
		case 5:
			line = 3;
			p_offset = 0;
			m_offset = 6;
			break;
		case 6:
			line = 3;
			p_offset = 2;
			m_offset = 0;
			break;
		default:
			p_offset = 0;
			m_offset = 0;
			break;

	}

	 if ( blink_time++ > 20 ) {
		 blink_flag = 1 - blink_flag;
		 blink_time = 0;
		 LED_ONOFF();
	 }

	if ( p_offset )	code = (uint8_t *)  &lcd_data[line++][0];
	if ( m_offset ) {
		code1 = (uint8_t *)  &lcd_data[line][0];
	}
	if ( p_offset ) line--;

	buf = lcd_buf;
	for (i=0; i< MAX_X; i++) {
		no_font = 1;
		if ( p_offset != 0 ) {
			 if ( *code < 32 || *code > 127 ) {
				 if ( *code >=8 && *code <= 15) {
					 font = &user_font[(*code)-8][0];
					 no_font = 0;
				 }

			 }
			 else {
				 font = &Font[ 5 * (*code - 32) ];
				 no_font = 0;
			 }
		}

		 for (j = 0; j < 5; j++) {
			 if ( no_font ) {
				 *buf++ =0;
			 }
			 else {
				 *buf++ = (*font) >> (8-p_offset)  ;
				 font++;
			 }
		 }

		 if ( cursor_sts == CURSOR_ON ) {
			 if ( cursor_y == line && cursor_x == i ){
				 buf -=5;
				 for (j = 0; j < 5; j++) {
					 if (blink_flag ) {
						 *buf++ |= (0x80 >> (8-p_offset));
					 }
					 else *buf++ &= ~(0x80 >> (8-p_offset));
				 }
			 }
		}

		*buf++ = 0;

		no_font = 1;
	    if ( m_offset != 0 ) {
	    	 buf -= 6;

	    	 if ( *code1 < 32 || *code1 > 127 ) {
				 if ( *code1 >=8 && *code1 <= 15) {
					 font = &user_font[(*code1)-8][0];
					 no_font = 0;
				 }
			 }
			 else {
				 font = &Font[ 5 * (*code1 - 32) ];
				 no_font = 0;
			 }
			 for (j = 0; j < 5; j++) {
				 if ( !no_font) {
					  *buf++ |= (*font) << ( 8 - m_offset)  ;
					 font++;
				 }
				 else buf++;
			 }
			 *buf++ = 0;
	    }


		 code++;
		 code1++;
	}
}



void LCD_line_bitmap(uint8_t line)
{
	uint8_t *code, *buf;
	uint8_t const  *font;
	uint8_t i,j;

	code = (uint8_t *)  &lcd_data[line][0];
	buf = lcd_buf;
	for (i=0; i< MAX_X; i++) {
		 if ( *code < 32 || *code > 127 ) {
			 if ( *code >=8 && *code <= 15) font = &user_font[(*code)-8][0];
			 else {	// non displayable character
				 for (j = 0; j < 6; j++) {
					 *buf++ = 0;
				 }
				 code++;
				 continue;
			 }
		 }
		 else font = &Font[ 5 * (*code - 32) ];
		 for (j = 0; j < 5; j++) {
			 *buf++ = *font++;
		 }
		 *buf++ = 0;

		 if ( cursor_sts == CURSOR_ON ) {
			 if ( cursor_y == line && cursor_x == i ){
				 buf -=6;
				 for (j = 0; j < 5; j++) {
					 *buf++ |= 0x80;
				 }
			 }
		 }
		 code++;
	}
}


unsigned char bit_reverse(unsigned char val)
{
 	int i, b;
	unsigned char tmp = 0;

	for ( i = 7, b = 0; i > 0; i-=2) {
		tmp |= (val << i) & (0x80 >> b);
		tmp |= (val >> i) & (0x01 << b++);
	}
//	printf("V=%x,R=%x ", val, tmp);
	return tmp;
}


void LCD_update()
{
	uint8_t  x;
	static int8_t	row = 1;

	lcd_control(LCD_START_LINE_0);

#if 0
		lcd_control(LCD_PAGE_ADDR_0 ); //page address
		lcd_control(LCD_COL_ADDR_H_0); // col. addr high
		lcd_control(LCD_COL_ADDR_L_0); // col. addr low
		for(x=0;x<128;x++)
		{
			lcd_data(0x00);
		}

#endif


		LCD_row_bitmap(row);
		lcd_control(LCD_PAGE_ADDR_0 + row); //page address
		lcd_control(LCD_COL_ADDR_H_0); // col. addr high
		lcd_control(LCD_COL_ADDR_L_0); // col. addr low
		for(x=0;x<128;x++)		{
			if ( x < 4 || x >= 124 ) lcd_data(0);
			else  {
				lcd_data(lcd_buf[x-4]);
			}			//if ( i == 7) *pbuf++ = tmp;
		}

		row++;
		if ( row > 6) row = 1;

#if 0
	lcd_control(LCD_PAGE_ADDR_0+7 ); //page address
	lcd_control(LCD_COL_ADDR_H_0); // col. addr high
	lcd_control(LCD_COL_ADDR_L_0); // col. addr low
	for(x=0;x<128;x++)
	{
		lcd_data(0x00);
	}
#endif
}




















