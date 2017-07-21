/*
 * lcd.h
 *
 *  Created on: 2017. 2. 15.
 *      Author: ksyoo
 */

#ifndef LCD_H_
#define LCD_H_


#define LCD_SPI

#define	LCD_CONTROL_DIR	P3DIR 	//P3DIR
#define	LCD_CONTROL	P3OUT		//P3OUT
#define	LCD_CS		BIT2
#define LCD_A0  	BIT3	// P3.3
#define LCD_WR		BIT5	// P3.5
#define LCD_RD		BIT4	// P3.4

#define	LCD_D0_DIR	P2DIR	// P2.0
#define	LCD_D0_OUT	P2OUT	// P2.0
#define	LCD_D0_IN	P2IN	// P2.0

#define	LCD_D0		BIT0	// P2.0
#define	LCD_D1		BIT1	// P2.1
#define	LCD_D2		BIT2	// P2.2
#define	LCD_D3		BIT3	// P2.3
#define	LCD_D4		BIT4	// P2.4
#define	LCD_D5		BIT5	// P2.5

#define	LCD_D6_DIR	P1DIR	// P2.0
#define	LCD_D6_OUT	P1OUT	// P2.0
#define	LCD_D6_IN	P1IN	// P2.0
#define	LCD_D6		BIT5	// P1.5
#define	LCD_D7		BIT7	// P1.7

#define LCD_LIGHT_DIR P3DIR	// P3DIR
#define LCD_LIGHT_OUT P3OUT	// P3OUT
#define LCD_LIGHT	 BIT0	//BIT0


#define NOP()	__no_operation();
#define LCD_CONTROL_ON	{ LCD_CONTROL |= (LCD_RD | LCD_WR); LCD_CONTROL &= ~(LCD_A0|LCD_CS); }
#define LCD_DATA_ON		{ LCD_CONTROL |= (LCD_RD | LCD_WR|LCD_A0); LCD_CONTROL &= ~LCD_CS; }

#define LCD_CONTROL_OFF	{ LCD_CONTROL |= LCD_CS; }
#define LCD_DATA_OFF	{ LCD_CONTROL |= LCD_CS; }

#define LCD_BUSY()

#define LCD_DISPLAY		0xAE
#define LCD_START_LINE	0x40

#define LCD_SET_PAGE	0xB0
#define LCD_SET_UPPER	0x10	// column address set upper bit
#define LCD_SET_LOWER	0x00	// column address set lower bit
#define LCD_READ_STATUS	0x00
#define LCD_SEL_ADC		0xA0
#define LCD_DISP_NORMAL	0xA6	// display normal/ reverse
#define LCD_DISP_ALL	0xA4	// display all points
#define LCD_SET_BIAS	0xA2
#define LCD_INC_ADDR	0xe0	// read/modify/write
#define LCD_NO_INC_ADDR 0xEE	// End
#define LCD_RESET		0xE2
#define LCD_SET_DIR		0xC0	// common output mode select
#define LCD_SET_POWER	0x28	// power control set
#define LCD_SET_RATIO	0x20	// V5 voltage regulator internal resistor ratio set
#define LCD_SET_VOLUME	0x81	// Electronic volume mode set
#define LCD_SET_VOL_REG	0x00	// electronic volume register set
#define LCD_SET_SLEEP	0xAC	// sleep mode set
#define LCD_SET_BOOSTER	0xf8	// booster ratio set
#define LCD_NOP			0xe3
#define LCD_TEST		0xf0


#define LCD_DISPLAY_ON	0xAF
#define LCD_DISPLAY_OFF	0xAE

#define LCD_START_LINE_0	0x40	// line addr = 0 ~ 63
#define LCD_PAGE_ADDR_0		0xB0	// page addr = 0 ~ 7
#define LCD_COL_ADDR_H_0		0x10	//  A7 A6 A5 A4
#define LCD_COL_ADDR_L_0		0x00	// A3 A2 A1 A0

#define LCD_ADC_NORMAL		0xA0		// NORMAL
#define LCD_ADC_REVERSE		0xA1		// REVERSE

#define LCD_DISP_NORMAL		0xA6		// NORMAL
#define LCD_DISP_REVERSE	0xA7		// REVERSE

#define LCD_ALL_ON			0xA5		// all points on
#define LCD_ALL_NORMAL		0xA4		// normal display mode

#define LCD_BIAS_0		0xA2	// USE 1/9 BIAS
#define LCD_BIAS_1		0xA3


#define LCD_COM_NORMAL	0xc0	// USE 1/65 DUTY

#define LCD_POWER_CONTROL	0x28
#define LCD_POWER_CONTROL_BOOST	0x04	// BOSTER ON
#define LCD_POWER_CONTROL_REG	0x02	// REGULATOR ON
#define LCD_POWER_CONTROL_FOL	0x01	// FOLLOWER ON
#define LCD_POWER_CONTROL_ALL	0x02F

#define LCD_BOOST_MODE	0xf8			// booster ratio select mode set
#define LCD_BOOST_RATIO_0	0x00		// use 4x

#define LCD_STATIC_IND	0xAC			// static indecator
#define LCD_STATIC_IND_OFF 0			// FLASH OFF
#define LCD_STATIC_IND_ON 1				// FLASH ON

#define LCD_DELAY	__delay_cycles(75);

#define MAX_X	20
#define MAX_Y	4
#define	 CH_ESC	0x1B
#define CH_CLEAR	0x43
#define CH_CON		0x53
#define CH_COFF		0x73
#define CH_LON		0x42
#define CH_LOFF		0x62
#define CH_00		0x48

#define CH_POS		0x4c
#define CH_FONT		0x44
#define CH_CHARS	0x4B
#define CH_PRINT	0x80

#define CMD_IDLE	0
#define CMD_WAIT	1
#define CMD_POS_X	2
#define CMD_POS_Y	3
#define CMD_FONT	4
#define CMD_FONT_CODE	5
#define CMD_FONT_DATA	6
#define CMD_CHARS	7
#define CMD_CHARS_X1	8
#define CMD_CHARS_Y1	9
#define CMD_CHARS_X2	10
#define CMD_CHARS_Y2	11
#define CMD_CHARS_DATA	12
#define CMD_CHARS_END	13

#define CURSOR_ON		0x01
#define CURSOR_OFF		0x00
#define CURSOR_BLINK	0x02
#define LIGHT_ON	1
#define LIGHT_OFF	0
#define LCD_ON		1
#define LCD_OFF		0

#ifdef LCD_SPI
#define lcd_data(a)	LCD_data(a)
#define lcd_control(a)	LCD_command(a)
#define lcd_data_buf(a, b) LCD_data_bytes(a, b)
#else
#define lcd_data(a)	LCD_write_data(a)
#define lcd_control(a)	LCD_write_control(a)
#define lcd_data_buf(a, b) LCD_wrtie_buf(a, b)
#endif

typedef struct {
	uint8_t	code;
	uint8_t	font[8];
} FONT_TYPE;

void LCD_write_control(uint8_t cmd);
void LCD_write_control_data(uint8_t cmd, uint8_t data);
void LCD_write_data(uint8_t data);
void LCD_write_buf(uint8_t *data, uint8_t size);
uint8_t LCD_read_status();
uint8_t LCD_read_data();
void LCD_update();
void LCD_light_onoff(uint8_t mode);
void LCD_init();

uint8_t LCD_command(uint8_t data);
uint8_t LCD_data_bytes(uint8_t *buf, int length);
uint8_t LCD_data(uint8_t data);

#endif /* LCD_H_ */
