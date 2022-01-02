/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.
	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:
	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.
	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.
	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devMAXREFDES117.h"

extern volatile WarpI2CDeviceState	deviceMAXREFDES117State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void
initMAXREFDES117()
{
  	deviceMAXREFDES117State.operatingVoltageMillivolts = 1800;
	PORT_HAL_SetMuxMode(PORTA_BASE, 5u, kPortMuxAsGpio);	//set input for INT pin

	return;  
}

WarpStatus
writeSensorRegisterMAXREFDES117(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	i2c_device_t slave =
	{
		.address = I2C_WRITE_ADDR,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMAXREFDES117State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterMAXREFDES117(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	
	i2c_device_t slave =
	{
		.address = I2C_READ_ADDR,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMAXREFDES117State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

uint32_t
MAXREFDES117_read_fifo()
{
  uint32_t red_ir;
  uint16_t red, ir;

  //read and clear status register
  readSensorRegisterMAXREFDES117(REG_INTR_STATUS_1, 1);
  readSensorRegisterMAXREFDES117(REG_INTR_STATUS_2, 1);

  readSensorRegisterMAXREFDES117(REG_FIFO_DATA, 6);	//read 6 bytes from FIFO

  red =  (0xFFFF) & ((deviceMAXREFDES117State.i2cBuffer[0]<<14)+(deviceMAXREFDES117State.i2cBuffer[1]<<6)+(deviceMAXREFDES117State.i2cBuffer[2]>>2)); //16 MSB of data
  ir  =  (0xFFFF) & ((deviceMAXREFDES117State.i2cBuffer[3]<<14)+(deviceMAXREFDES117State.i2cBuffer[4]<<6)+(deviceMAXREFDES117State.i2cBuffer[5]>>2));

  red_ir = ((uint32_t)red << 16) + ir;	//combine red and ir to return
  return red_ir;
}

WarpStatus
configureSensorMAXREFDES117()
{
	warpScaleSupplyVoltage(deviceMAXREFDES117State.operatingVoltageMillivolts);

	writeSensorRegisterMAXREFDES117(REG_INTR_ENABLE_1,0xc0); // INTR setting - data ready and almost full
  	writeSensorRegisterMAXREFDES117(REG_INTR_ENABLE_2,0x00);
  	writeSensorRegisterMAXREFDES117(REG_FIFO_WR_PTR,0x00); //FIFO_WR_PTR[4:0]
  	writeSensorRegisterMAXREFDES117(REG_OVF_COUNTER,0x00);  //OVF_COUNTER[4:0]
  	writeSensorRegisterMAXREFDES117(REG_FIFO_RD_PTR,0x00);  //FIFO_RD_PTR[4:0]
  	writeSensorRegisterMAXREFDES117(REG_FIFO_CONFIG,0x4f);  //sample avg = 4, fifo rollover=false, fifo almost full = 17
  	writeSensorRegisterMAXREFDES117(REG_MODE_CONFIG,0x03);   //0x03 for SpO2 mode
  	writeSensorRegisterMAXREFDES117(REG_SPO2_CONFIG,0x23);  // SPO2_ADC range = 4096nA, SPO2 sample rate (50 Hz), LED pulseWidth (400uS)
  	writeSensorRegisterMAXREFDES117(REG_LED1_PA,0x24);   //Choose value for ~ 7mA for LED1
  	writeSensorRegisterMAXREFDES117(REG_LED2_PA,0x24);   // Choose value for ~ 7mA for LED2
  	writeSensorRegisterMAXREFDES117(REG_PILOT_PA,0x7f);   // Choose value for ~ 25mA for Pilot LED
  	return true;
}

bool
MAXREFDES117_read_INT()
{
	if(GPIO_DRV_ReadPinInput(kWarpPinMAXREFDES117_INT)) //read input from INT
		return true;
	return false;
}

