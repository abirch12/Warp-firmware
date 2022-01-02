#include <stdint.h>

#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 11),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");


/*	Add text as headings for measurements	*/
	uint16_t col = 8;    //start point
	uint16_t row = 9;
	int i;

	char spo2_text[] = "Sp02 - ";
	for (i=0;i<7;i++){
		drawChar(col,row,(char)spo2_text[i]);
		col += 8;
	}
	
	col = 24;    //HR line
	row = 29;
	char hr_text[] = "HR - ";
	for (i=0;i<5;i++){
		drawChar(col,row,(char)hr_text[i]);
		col += 8;
	}

	return 0;
}

void 
drawLine (uint16_t x, uint16_t y, uint16_t scol, uint16_t srow, uint16_t ecol, uint16_t erow){
	writeCommand(kSSD1331CommandDRAWLINE);
	writeCommand(x+scol);	//start point
	writeCommand(y+srow);
	writeCommand(x+ecol);	//end point
	writeCommand(y+erow);
	writeCommand(0x00);	//colour
        writeCommand(0x00);
        writeCommand(0x28);
}

void
drawChar (uint16_t col, uint16_t row, char c){
	switch (c){		//Only 0-9, S, p, H, R, - and space are available characters
		case 'S':
			drawLine(col,row,1,0,5,0);
			drawLine(col,row,7,2,5,0);
			drawLine(col,row,0,1,0,4);
			drawLine(col,row,0,4,7,5);
			drawLine(col,row,7,5,7,8);
			drawLine(col,row,6,9,2,7);
			break;
		case 'p':
			drawLine(col,row,2,3,2,9);
			drawLine(col,row,2,3,5,3);
			drawLine(col,row,2,6,5,6);
			drawLine(col,row,6,4,6,5);
			break;
		case 'H':
			drawLine(col,row,0,0,0,9);
			drawLine(col,row,7,0,7,9);
			drawLine(col,row,0,5,7,5);
			break;
		case 'R':
			drawLine(col,row,1,0,1,9);
			drawLine(col,row,1,0,6,0);
			drawLine(col,row,1,5,6,5);
			drawLine(col,row,7,1,7,4);
			drawLine(col,row,3,5,7,9);
			break;
		case ' ':
			break;
		case '-':
			drawLine(col,row,1,5,6,5);
			break;
		case '0':
			drawLine(col,row,1,1,1,8);
			drawLine(col,row,6,1,6,8);
			drawLine(col,row,2,0,5,0);
			drawLine(col,row,2,9,5,9);
			break;
		case '1':
			drawLine(col,row,3,0,3,9);
			break;
		case '2':
			drawLine(col,row,1,0,5,0);
			drawLine(col,row,6,1,6,3);
			drawLine(col,row,5,4,2,4);
			drawLine(col,row,1,5,1,8);
			drawLine(col,row,2,9,6,9);
			break;
		case '3':
			drawLine(col,row,1,0,5,0);
			drawLine(col,row,6,1,6,8);
			drawLine(col,row,1,4,6,4);
			drawLine(col,row,1,9,5,9);
			break;
		case '4':
			drawLine(col,row,1,0,1,4);
			drawLine(col,row,6,0,6,9);
			drawLine(col,row,1,4,6,4);
			break;
		case '5':
			drawLine(col,row,2,0,6,0);
			drawLine(col,row,1,1,1,3);
			drawLine(col,row,2,4,5,4);
			drawLine(col,row,6,5,6,8);
			drawLine(col,row,1,9,5,9);
			break;
		case '6':
			drawLine(col,row,2,0,5,0);
			drawLine(col,row,1,1,1,8);
			drawLine(col,row,1,4,5,4);
			drawLine(col,row,6,5,6,8);
			drawLine(col,row,2,9,5,9);
			break;
		case '7':
			drawLine(col,row,1,0,6,0);
			drawLine(col,row,6,0,6,9);
			break;
		case '8':
			drawLine(col,row,2,0,5,0);
			drawLine(col,row,1,1,1,8);
			drawLine(col,row,6,1,6,8);
			drawLine(col,row,1,4,6,4);
			drawLine(col,row,2,9,5,9);
			break;
		case '9':
			drawLine(col,row,2,0,5,0);
			drawLine(col,row,1,1,1,3);
			drawLine(col,row,2,4,6,4);
			drawLine(col,row,6,1,6,9);
			break;
	}
}

void 
newReading(int32_t spo2, int32_t heart_rate){
	uint16_t col = 64;
	uint16_t row = 9;
	int32_t hundred, ten, unit;

	writeCommand(kSSD1331CommandCLEAR);	//clear to the right of headings
	writeCommand(0x40);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);

	if (spo2 > 99){
		drawChar(col,row,'1');
		col += 8;
		drawChar(col,row,'0');
		col += 8;
		drawChar(col,row,'0');
	}
	else{
		ten = spo2/10;			//finding digits of value
		unit= spo2 - (10*ten);
		drawChar(col,row,(char)(ten+'0'));
		col += 8;
		drawChar(col,row,(char)(unit+'0'));
	}

	col = 64;
	row = 29;
	hundred = 0;
	if (heart_rate > 99){
		hundred = 1;
		drawChar(col,row,(char)(hundred+'0'));
		col += 8;
	}
	ten = (heart_rate-(100*hundred))/10;
	unit= heart_rate - ((100*hundred) + (10*ten));
	drawChar(col,row,(char)(ten+'0'));
	col += 8;
	drawChar(col,row,(char)(unit+'0'));
}
