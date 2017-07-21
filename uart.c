/*
 * uart.c
 *
 *  Created on: 2015. 10. 8.
 *      Author: Administrator
 */

#include	<msp430g2433.h>
#include	<stdio.h>
#include 	<stdint.h>		// defines integer bits types
#include	"io.h"
#include	"lcd.h"

#define RxBuffSize	80		// 8
#define TxBuffSize  8		//ksyoo 100 // 200

byte RxBuff0[RxBuffSize];
byte TxBuff0[TxBuffSize];
byte RxInPtr0=0;
byte RxOutPtr0=0;
word TxInPtr0=0;
word TxOutPtr0=0;
byte TxComplete0 = 0;
byte RxError0 = 0;
byte TxFlag0 = 0;
byte RxFlag0 = 0;

// ksyoo
byte UART1_status =0;
#define	UART_BUF_FULL	1
#define	UART_BUF_UNDER_RUN	2
#define	UART_BUF_EMPTY	3

#define Disable_SIO_Intr0()	(IE2 |= UCA0TXIE + UCA0RXIE)
#define Enable_SIO_Intr0() 	(IE2 & = ~(UCA0TXIE + UCA0RXIE))


FONT_TYPE	rx_font;
uint8_t user_font[8][5];
uint8_t	font_count, char_count;

uint8_t	cmd_state = CMD_WAIT;
uint8_t	user_x, user_y, user_data[2 * MAX_X];

uint8_t	cursor_x, cursor_y, cursor_sts, lcd_data[MAX_Y][MAX_X], lcd_mode, light_sts;


void set_cursor()
{
	cursor_x = user_x;
	cursor_y = user_y;
}

void set_light()
{
	LCD_light_onoff(light_sts);
}


void lcd_data_clear()
{
	uint8_t i;
	uint8_t *p;

	p = (uint8_t *) &lcd_data[0][0];
	for ( i= 0; i < MAX_Y * MAX_X; i++) {
		*p++ = ' ';	// SPACE
	}
}

void	init_lcd_mode()
{
	uint8_t i, *buf;

	cursor_x= cursor_y = 0;
	cursor_sts = CURSOR_ON;
	light_sts = LIGHT_ON;
	lcd_mode	= LCD_ON;
	cmd_state = CMD_IDLE;

	lcd_data_clear();

	buf = &user_font[0][0];
	for ( i= 0; i < 8*5; i++) {
		*buf++ = 0;	// SPACE
	}
}

void lcd_clear()
{
	//Uart_TxStr("CLEAR");
	lcd_data_clear();
	LCD_update();
}

void cursor_onoff(uint8_t mode)
{
	//Uart_TxStr("CURSOR");
	cursor_sts = mode;
}

void light_onoff(uint8_t mode)
{
	LCD_light_onoff(mode);
}

void cursor_zero()
{
	//Uart_TxStr("ZERO");
	cursor_x = cursor_y = 0;
}

void cursor_xy()
{
	//Uart_TxStr("C_XY");

	if (  user_x >= 20 || user_y >= 4 )	return;
	cursor_x = user_x;
	cursor_y = user_y;
}

void set_user_font()
{
	uint8_t i, j, *rf, *uf;

	//Uart_TxStr("FONT");
	i = rx_font.code - 8;

	uf = &user_font[i][0];

	for ( i = 0; i < 5; i++) {
		rf = rx_font.font;
		*uf = 0;
		for (j = 0; j < 8; j++) {
			if ( *rf & (0x10 >> i )) {
				*uf |=  (0x01 << j );
			}
			rf++;
		}
		uf++;
	}
}

void display_char(uint8_t ch)		// dispay (x, y) -> lcd_data[y][x]
{
	uint8_t x, y;

	//Uart_TxChar('$');
	//Uart_TxChar(ch);

	x = cursor_x;
	y = cursor_y;

	lcd_data[y][x] = ch;
	x++;
	if ( x >= MAX_X ) {
		x = 0;
		y++;
	}
	if ( y >= MAX_Y ) {
		y = 0;
	}

	cursor_x = x;
	cursor_y = y;
	LCD_update();
}


void display_chars()		// dispay (x, y) -> lcd_data[y][x]
{
	uint8_t x, y, i;

	//Uart_TxStr("CHARS");

	if (  user_x < 20 ) x = user_x;
	else x = cursor_x;

	if ( user_y < 4 )	y = user_y;
	else y = cursor_y;

	i = 0;
	if (char_count > 40) char_count = 40;
	while ( char_count-- ) {
		lcd_data[y][x] = user_data[i++];
		x++;
		if ( x >= MAX_X ) {
			x = 0;
			y++;
		}
		if ( y >= MAX_Y ) {
			y = 0;
			break;
		}
	}
	cursor_x = x;
	cursor_y = y;

	LCD_update();
}

void lcd_function(uint8_t cmd)
{
	switch ( cmd ) {
		case CH_CLEAR:
			lcd_clear();
			break;
		case CH_CON:
			cursor_onoff(CURSOR_ON);
			break;
		case CH_COFF:
			cursor_onoff(CURSOR_OFF);
			break;
		case CH_LON:
			light_onoff(LIGHT_ON);
			break;
		case CH_LOFF:
			light_onoff(LIGHT_OFF);
			break;
		case CH_00:
			cursor_zero();
			break;
		case CH_POS:
			cursor_xy();
			break;
		case CH_FONT:
			set_user_font();
			break;
		case CH_CHARS:
			display_chars();
			break;
		default:
			if ( cmd != 0 ) {
				display_char(cmd & 0x7f);
			}
			break;
	}
}


uint8_t get_cmd()
{
	uint8_t ret = 0;
	uint8_t rsize;
	uint8_t rd, num[8];

	rsize = Uart_RxQueue();

	while ( rsize-- > 0 ) {
		rd = Uart_RxChar_Buf();

		//itoa(rd, num);
		//Uart_TxStr(num);
		//Uart_TxChar(rd);
		switch ( cmd_state) {
		case CMD_IDLE :
			if ( rd == CH_ESC ) cmd_state = CMD_WAIT;
			else  if ( rd < 0x80 ) ret = rd | CH_PRINT;
			break;
		case CMD_WAIT:
			switch (rd ) {
			case CH_CLEAR:
			case CH_CON:
			case CH_COFF:
			case CH_LON:
			case CH_LOFF:
			case CH_00:
				ret = rd;
				break;
			case CH_POS:
				cmd_state = CMD_POS_X;
				break;
			case CH_FONT:
				cmd_state = CMD_FONT_CODE;
				font_count = 0;
				break;
			case CH_CHARS:
				cmd_state = CMD_CHARS_X1;
				char_count = 0;
				break;
			default:
				cmd_state = CMD_IDLE;
				break;
			}
			break;
		case CMD_POS_X:
			if ( rd > 20 ) {
				cmd_state = CMD_IDLE;
				break;
			}
			user_x = rd;
			cmd_state = CMD_POS_Y;
			break;
		case CMD_POS_Y:
			if ( rd > 20 ) {
				cmd_state = CMD_IDLE;
				break;
			}
			user_y = rd;
			ret = CH_POS;
			break;
		case CMD_FONT_CODE:
			if ( rd < 8 || rd > 15  ) {
				cmd_state = CMD_IDLE;
				break;
			}
			rx_font.code = rd;
			cmd_state = CMD_FONT_DATA;
			font_count = 0;
			break;
		case CMD_CHARS_X1:
			if ( rd > 20 ) {
				cmd_state = CMD_IDLE;
				break;
			}
			user_x = rd;
			cmd_state = CMD_CHARS_Y1;
			break;
		case CMD_CHARS_Y1:
			if ( rd > 20 ) {
				cmd_state = CMD_IDLE;
				break;
			}
			user_y = rd;
			cmd_state = CMD_CHARS_X2;
			break;
		case CMD_CHARS_X2:
			if ( rd != user_x ) {
				cmd_state = CMD_IDLE;
				break;
			}
			cmd_state = CMD_CHARS_Y2;
			break;
		case CMD_CHARS_Y2:
			if ( rd != user_y ) {
				cmd_state = CMD_IDLE;
				break;
			}
			cmd_state = CMD_CHARS_DATA;
			break;
		case CMD_FONT_DATA:
			rx_font.font[font_count++] = rd;
			if ( font_count >= 8) {
				ret  = CH_FONT;
			}
			break;
		case CMD_CHARS_DATA:
			user_data[char_count++] = rd;
			if ( rd == 0x00 ) {				//cmd_state = CMD_CHARS_END;
				char_count--;
				ret = CH_CHARS;
			}
			else if ( char_count >= 80 ) cmd_state = CMD_IDLE;
			break;
		default:
			cmd_state = CMD_IDLE;
			break;
		}

		if ( ret != 0 ) {
			cmd_state = CMD_IDLE;
			break;
		}
	}
	return ret;
}

/*
#define Disable_SIO_Intr()	asm("cli")
#define Enable_SIO_Intr() 	asm("sei")
*/


//******************************************************************************
//   MSP430G2xx3 Demo - USCI_A0, 115200 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1MHz = 1MHz/115200 = ~8.7
//   ACLK = n/a, MCLK = SMCLK = CALxxx_1MHZ = 1MHz
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.2/UCA0TXD|------------>
//            |                 | 115200 - 8N1
//            |     P1.1/UCA0RXD|<------------
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************

void	 UART_init()
{

	P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	P1SEL2 |= BIT1 + BIT2;
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
#ifndef CLK_16M
	UCA0BR0 = 8;                              // 1MHz 115200
	UCA0BR1 = 0;
	UCA0MCTL = UCBRS2 + UCBRS0;
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

	RxInPtr0 = RxOutPtr0 = 0;
	TxInPtr0 = TxOutPtr0 = 0;

	IE2 |= UCA0RXIE + UCA0TXIE;
#else
	UCA0BR0 = 138;                              // 138@16MHz 115200,  8 @1MHz 115200
	UCA0BR1 = 0;                              //  0 @16MHz 115200,  0 @1MHz 115200

	UCA0MCTL = UCBRS2 + UCBRS1 + UCBRS0;               // 7 for 16MHz/115200,  Modulation UCBRSx = 5
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

	RxInPtr0 = RxOutPtr0 = 0;
	TxInPtr0 = TxOutPtr0 = 0;

	IE2 |= UCA0RXIE + UCA0TXIE;               // Enable USCI_A0 RX/TX interrupt
#endif

   __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled

}

#if 0

char udata;

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  //while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
  udata = UCA0RXBUF;                    // TX -> RXed character
  IFG2 |=  UCA0TXIFG;
  //LED_ONOFF();
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
  static int count=0;

  count++;
  if ( count > 2 ) {
	  count = 0;
	  LED_ONOFF();
  }
  UCA0TXBUF = udata;                       // Read, justify, and transmit
  while (!(IFG2&UCA0TXIFG));
  IFG2 &=  ~UCA0TXIFG;
}

#endif

#if 1
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{

	LED2_ON();
#if 0
	if ( IFG2 & UCB0RXIFG ) {
		SPI_RX_ISR();
		//return;			// USCIB0RXIF -> spi_tx_isr()
	}
#endif

	if (  IFG2 & UCA0RXIFG ) {	// UCA0RXI : UART interrupt
		RxBuff0[RxInPtr0++]= UCA0RXBUF;
		if (RxInPtr0 >= RxBuffSize)
		{
			if (!RxOutPtr0) 	RxInPtr0--;
			else 				RxInPtr0 = 0;
		}
		else if (RxInPtr0 == RxOutPtr0)
		{
			RxInPtr0--;
		}
		RxFlag0 = 1;
	}
	LED2_OFF();
}

#ifndef USE_POLLING_TX
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	// LED_ON();

#if 0
	 if ( IFG2 & UCB0TXIFG ) {			// USB0TX interrupt : SPI
		 IFG2 &=  ~UCB0TXIFG;
	 }
#endif

	 if ( IFG2 & UCA0TXIFG ) {		 	// UCA0TX interrupt : UART
		if(TxOutPtr0 != TxInPtr0)
		{
			UCA0TXBUF = TxBuff0[TxOutPtr0++];
			if (TxOutPtr0 >= TxBuffSize)
				TxOutPtr0 = 0;
		}
		else
		{
			while (!(IFG2 & UCA0TXIFG));
			IFG2 &=  ~UCA0TXIFG;
			TxFlag0 = 0;
		}
	 }

	// LED_OFF();
}
#else
void USCI0TX_ISR(void)
{
	if(TxOutPtr0 != TxInPtr0)
	{
		UCA0TXBUF = TxBuff0[TxOutPtr0++];
		if (TxOutPtr0 >= TxBuffSize)
			TxOutPtr0 = 0;
	}
	else
	{
		TxFlag0 = 0;
	}
}

#endif

#endif

void Uart_TxChar(unsigned char cdata)
{
	__disable_interrupt();
	TxBuff0[TxInPtr0++] = cdata;
	if (TxInPtr0 >= TxBuffSize)
		{
			if (!TxOutPtr0)	TxInPtr0--;
			else			TxInPtr0 = 0;
		}
	else if (TxInPtr0 == TxOutPtr0)
	{
			TxInPtr0--;
	}
	if (!TxFlag0)
		{
			TxFlag0 =1;
			TxComplete0 =0;
#ifndef USE_POLLING_TX
			IFG2 |= UCA0TXIFG;
 			//UCSR0B	|=	(1<<UDRIE0) | (1<<TXCIE0);								// RX Complete enable..
#endif
		}
	__enable_interrupt();
}


void Uart_TxStr(char *str)
{
	while (*str) Uart_TxChar(*str++);
}

void Uart_TxBuf(char *str, short size)
{
	while (size--) Uart_TxChar(*str++);
}

byte Uart_RxChar(void)
{
	byte rxd;

	while (!Uart_RxQueue());
	rxd = RxBuff0[RxOutPtr0++];
	if (RxOutPtr0>=RxBuffSize) RxOutPtr0=0;
	return (rxd);
}


byte Uart_RxQueue(void)
{
	if ( RxInPtr0 < RxOutPtr0 )
		return ( RxBuffSize + RxInPtr0 - RxOutPtr0 );
	else
		return ( RxInPtr0 - RxOutPtr0 );
}



byte Uart_RxChar_Buf()
{
	byte rxd;

	rxd = RxBuff0[RxOutPtr0++];
	if (RxOutPtr0 >= RxBuffSize) RxOutPtr0 = 0;
	return (rxd);

}




