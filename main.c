#include  	<msp430g2433.h>
#include <stdint.h>
#include	"io.h"
#include 	"hmain.h"

void main(void)
{
 	WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer

	init_control();
	hmain();
}

