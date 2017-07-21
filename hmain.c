#include  <msp430g2433.h>

#include 	<stdint.h>		// defines integer bits types
#include "io.h"
#include "hmain.h"
#include "lcd.h"

#define	POWER_WAIT	3200	// 20sec = 10msec * 2000

// power_state

#define	POWER_ON	0x01
#define	POWER_OFF	0x02
#define CPU_ON		0x01
#define	CPU_OFF		0x02
#define	CPU_OFFON		0x03

#define	FLASH_ADDR	0xf000

extern uint16_t timerA_count, lapse ;
extern uint8_t blink_time;
//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------

#define	print		// TimerA_UART_print

void	itoa(short int no, char *str)
{
	short int digit , r = 0;

	for (digit = 3; digit >= 0; digit--){
		r = no & 0xf ;
		if ( r < 10 ) str[digit] = '0' + r;
		else str[digit] = 'a' + r - 10;
		 no = no >> 4;
	}
	str[4] = 0;
}

void	IO_init()
{
	//DCOCTL = 0x00;

#ifndef CLK_16M	// when use DCO
    //1Mhz
     BCSCTL1 = CALBC1_1MHZ;                    // Set range
     DCOCTL = CALDCO_1MHZ;                     // Set DCO step + modulation */
     BCSCTL3 |= XCAP_3;

   /* //8Mhz
     BCSCTL1 = CALBC1_8MHZ;                    // Set range
     DCOCTL = CALDCO_8MHZ;                     // Set DCO step + modulation */

   /* //12Mhz
     BCSCTL1 = CALBC1_12MHZ;                   // Set range
     DCOCTL = CALDCO_12MHZ;                    // Set DCO step + modulation*/
#else
    //16Mhz
     BCSCTL1 = CALBC1_16MHZ;                   // Set range
     DCOCTL = CALDCO_16MHZ;                    // Set DCO step + modulation*/
     BCSCTL3 |= XCAP_3;
     //BCSCTL2 = 0x00;

#endif
#if 0		// MSP430G2433 doesn't support HF
     // set for the external 16MHz osc
     BCSCTL1 = XT2OFF + XTS + RSEL3 + RSEL2 + RSEL1 + RSEL0;	// 0xCF
     BCSCTL2 = SELM1 + SELM0 + SELS;							// 0xC8
     BCSCTL3 = LFXT1S_0;										// 0x00 LFXT1 normal operation
#endif

    P1OUT = 0x00;
	P2OUT = 0x00;
	P3OUT = 0x00;

	P1SEL = 0;		// init PxSEL
	P1SEL2 = 0;		// init PxSEL2

#if 0	// it causes error to use ACLK
	//P2SEL = 0;		// init PxSEL
	//P2SEL2 = 0;		// init PxSEL2
#endif

	LED_DIR |= LED1 | BIT6;

	LED_OUT |= LED1 | BIT6;
	delay(10);
	LED_OUT &= ~ ( LED1 + BIT6);

}


void	init_control()
{
	IO_init();
	UART_init();
	SPI_init();
	TimerA_init();
	LCD_init();

}


void	delay(int wait)
{
	volatile int i= 0, j;
	while( wait--) {
		for(i= 0; i < 200; i++){
			j = i* wait;
		}
	}
}

void hmain(void)
{
#if 0	// in main.c
	WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer
	init_control();
#endif

	char cmd;
	//int ret;
	//char num[10];
	//int count = 0;

	init_lcd_mode();
	Uart_TxStr("YOOHA AX SENSOR");

	//while(1);
	while(1) {

		//LED_ON();
		cmd = get_cmd();			// get the command from UART port (MAIN UNIT)
		if ( cmd ) {
			lcd_function();
			//Uart_TxChar(0x0d);
		}

		if (   lapse >= 50 ) {		// timerA_count 1 = 0.5msec
			LCD_update();
			//LED_ONOFF();
			lapse = 0;
		}
#if 0
		//LED_OUT ^= LED1;
		delay(100);
		Uart_TxChar('G');
#endif


	}

}


