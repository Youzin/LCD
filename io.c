//*

//******************************************************************************

#include  <msp430g2433.h>

#include 	<stdint.h>		// defines integer bits types
#include	"io.h"
#include "hmain.h"
#include "lcd.h"


#define LCD_CS_ON		{ LCD_CONTROL &= ~LCD_CS; }
#define LCD_CS_OFF		{ LCD_CONTROL |= LCD_CS; }

#define LCD_A0_ON		{ LCD_CONTROL &= ~LCD_A0; }
#define LCD_A0_OFF		{ LCD_CONTROL |= LCD_A0; }

#define	TIMER_DIV	8000	// // 0.5msec = 16MHz / 8000
//#define	TIMER_DIV	16	// // 0.5msec ~= 16 / 32768;
uint16_t	timerA_count = 0;
uint16_t 	lapse = 0;
static word 	half_ms;	// for watchdog

unsigned int txData;                        // UART internal variable for TX
unsigned char rxBuffer;                     // Received UART character


// Test for valid RX and TX character
// Echo character
#if 0
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	char data;

  while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?
 // __delay_cycles(50);
  data= UCB0RXBUF;
  __delay_cycles(50);
  UCB0TXBUF = data;
  //LED_ONOFF();
}

#endif

#if 0
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  volatile unsigned int i;

  while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?

#if 0
  if (UCB0RXBUF == SLV_Data)                // Test for correct character RX'd
    LED_OUT |= LED1;                          // If correct, light LED
  else
    LED_OUT &= ~LED1;                         // If incorrect, clear LED
#endif

  MST_Data++;                               // Increment master value
  SLV_Data++;                               // Increment expected slave value
  UCB0TXBUF = MST_Data;                     // Send next value

  __delay_cycles(50);                     // Add time between transmissions to
}                                           // make sure slave can keep up
#endif

void SPI_RX_ISR(void)
{
  volatile unsigned int i;

  while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?

#if 0
  if (UCB0RXBUF == SLV_Data)                // Test for correct character RX'd
    LED_OUT |= LED1;                          // If correct, light LED
  else
    LED_OUT &= ~LED1;                         // If incorrect, clear LED
#endif

  //MST_Data++;                               // Increment master value
  //SLV_Data++;                               // Increment expected slave value
  //UCB0TXBUF = MST_Data;                     // Send next value

  __delay_cycles(10);                     // Add time between transmissions to
}

#if 1

void TimerA_init(void)
{

  TACCR0 = TIMER_DIV;							// 0.5msec
  TACTL = 0x0210; //TASSEL_1 + MC_1 ;	//0x0210; //TASSEL_2 + MC_1;                  // SMCLK, up mode
  TACCTL0 = CCIE;                             // CCR0 interrupt enabled
  //_BIS_SR(GIE);                 // Enter LPM0 w/ interrupt
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
  // read sensor data through SPI
	timerA_count++;
	if ( timerA_count > 0x8000 ) {
		timerA_count = 0;
		//LED_ONOFF();
	}
	lapse++;
	__disable_interrupt();

	__enable_interrupt();
}

#else
void TimerA_init(void)
{

  TACCR0 = 16;							// 0.5msec ~= 16 / 32768;
  TACTL = 0x0110; //TASSEL_1 + MC_1 ;	//0x0210; //TASSEL_2 + MC_1;                  // SMCLK, up mode
  TACCTL0 = CCIE;                             // CCR0 interrupt enabled
  //_BIS_SR(GIE);                 // Enter LPM0 w/ interrupt
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
  // read sensor data through SPI
	timerA_count++;
	if ( timerA_count > 1000 ) {
		timerA_count = 0;
		LED_ONOFF();
	}

	__disable_interrupt();
	if ( ax_read_flag == 1 ) {

		ax_data_status = spi_read_s(STATUS_REG);
		if ( (ax_data_status & AX_DATA_READY ) == AX_DATA_READY ) {
			ax_data_ready = AX_DATA_READY ;
		}
		else ax_data_ready = 0;
	}
	__enable_interrupt();

}
#endif

void	wait_ms(short msec)
{
	msec *= 2;
	while( msec-- ) {
		half_ms = 1;
		IE1 |= WDTIE;
		WDTCTL = WDT_MDLY_0_5;
		//while(half_ms);		// wait for expiring WDT+ interval
		_BIS_SR(LPM0_bits+GIE);         // Enter LPM0 w/ interrupts
	}
}

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
	half_ms = 0;
	//IE1 &= ~WDTIE;
	WDTCTL = WDTPW +  WDTHOLD;
	__bic_SR_register_on_exit(LPM0_bits);
}



/// Select which USCI module to use
#define SPI_USE_USCI    0 ///< \hideinitializer
/**<    0 = USCIA0 \n
*       1 = USCIA1 \n
*       2 = USCIA2 \n
*       3 = USCIA3 \n
*       4 = USCIB0 \n
*       5 = USCIB1 \n
*       6 = USCIB2 \n
*       7 = USCIB3
**/

/// Select which clock source to use
#define SPI_CLK_SRC        2 ///< \hideinitializer
/**<    1 = ACLK    \n
*       2 = SMCLK
**/

/// SPI Clock division. Must be 4 or greater
#define SPI_CLK_DIV        4 ///< \hideinitializer

/// Byte that is transmitted during read operations



unsigned char MST_Data, SLV_Data;

void SPI_init(void)
{
  volatile unsigned int i;

  //CS_OUT |= CS0;
  //CS_DIR |= CS0;                     //
  P1SEL |= MISO + MOSI + SCLK;
  P1SEL2 |= MISO + MOSI + SCLK;

  UCB0CTL1 = UCSWRST; 						// reset state machine

//  UCB0CTL0 = UCCKPL + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
  UCB0CTL0 = UCCKPL+ UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;                     // SMCLK
  UCB0BR0 = 16;                          	//  800KHz = 16MHz / 20
  UCB0BR1 = 0;                              //
  //UCB0STAT |= 0x80;

  UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//  IE2 |= UCB0RXIE;                          // Enable USCI0 RX interrupt

 // CS_OUT &= ~CS0;                           // Now with SPI signals initialized,
  __delay_cycles(75);                 		// Wait for slave to initialize
  //CS_OUT |= CS0;
}

void	spi_write( byte data)
{	//CS_LOW();

	UCB0TXBUF = data;    // write
    while (( IFG2 & UCB0TXIFG) == 0); 	// wait for tx buffer to be ready
    while (( UCB0STAT & UCBUSY) == 1); 	// wait for transfer to complete
    IFG2 &= ~UCB0RXIFG; // clear the rx flag
    //CS_HIGH();
}

#if 1


void DelayCS(void)
{
	unsigned int TempCyc =	30;		// 20 ~ 30;
	while(TempCyc--);
}

uint8_t LCD_data(uint8_t data)
{
  //LCD_A0_OFF;
  LCD_CS_ON;
  /* Loop while DR register in not emplty */
  spi_write(data);

  LCD_DELAY;
  DelayCS();

  LCD_CS_OFF;
  return 0;
}

uint8_t LCD_data_bytes(uint8_t *buf, int length)
{
	//LCD_A0_OFF;
	LCD_CS_ON;
	while( length--) {
		/* Loop while DR register in not emplty */
		spi_write(*buf++);
	}

	LCD_DELAY;
	DelayCS();
	LCD_CS_OFF;
	return 0;
}

uint8_t LCD_command(uint8_t data)
{
	LCD_CS_ON;
	LCD_A0_ON;

  /* Loop while DR register in not emplty */
  spi_write(data);

  LCD_DELAY;
  DelayCS();
  LCD_A0_OFF;
  LCD_CS_OFF;


  return 0;
}

#endif

