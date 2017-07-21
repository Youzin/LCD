/*
 * io.h
 *
 *  Created on: 2012. 11. 2.
 *      Author: Administrator
 */

#ifndef IO_H_
#define IO_H_

#define HW_TARGET
#define SW_I2C
#define LOOP_LED
#define CLK_16M

typedef unsigned char byte;
typedef	unsigned short	word;

#define SPI_UCCTL0      UCB0CTL0
#define SPI_UCCTL1      UCB0CTL1
#define SPI_UCBR        UCB0BRW
//#define SPI_UCMCTL      UCB0MCTL
#define SPI_UCSTAT      UCB0STAT
#define SPI_UCRXBUF     UCB0RXBUF
#define SPI_UCTXBUF     UCB0TXBUF
#define SPI_UCIE        UCB0IE
#define SPI_UCIFG       UCB0IFG
#define SPI_UCIV        UCB0IV

#define SPI_OUT	P1OUT
#define SPI_IN	P1IN
#define	SPI_DIR		P1DIR
#define SPI_SEL		P1SEL
#define SPI_SEL1	P1SEL2
#define	SCLK		BIT5	// P1.5 UCB0CLK
#define	MISO		BIT6	// P1.6 UCB0SOMI
#define	MOSI		BIT7	// P1.7 UCB0SIMO

#define	CS_OUT		P2OUT
#define	CS_DIR		P2DIR
#define	CS0			BIT3	// P2.3 Accelerometer Chip Select
#define	CS1			BIT4

#define CS_LOW()	 (CS_OUT &= ~CS0)
#define CS_HIGH()	 (CS_OUT |= CS0 )


#define	UART_TX_OUT	P1OUT
#define	UART_RX_IN	P1IN
#define	UART_DIR		P1DIR
#define	UART_RX			BIT1	//P1.1 UCA0RXD
#define	UART_TX			BIT2	//P1.2 UCA0TXD

#define	LED_OUT		P1OUT
#define LED_DIR		P1DIR
#define	LED1		BIT4
#define LED2		BIT6

#define LED_ON()	LED_OUT |= LED1
#define LED_OFF()	LED_OUT &= ~LED1
#define LED_ONOFF()	LED_OUT ^= LED1

#define LED2_ON()	LED_OUT |= LED2
#define LED2_OFF()	LED_OUT &= ~LED2
#define LED2_ONOFF()	LED_OUT ^= LED2

void	 UART_init();
void 	Uart_TxBuf(char *str, short size);
void 	Uart_TxStr(char *str);
void 	Uart_TxChar(unsigned char cdata);
byte 	Uart_RxQueue(void);
byte 	Uart_RxChar_Buf();
uint8_t	get_cmd();

void 	TimerA_init(void);
void	wait_ms(short msec);


#endif /* IO_H_ */
