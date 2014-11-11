#pragma once
//-----------------------------------------------------------------------------
// DNT900Registers.h
//
// This file contains the register definitions for the DNT900 sensor used in
// the DNT900Driver class
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// DNT900 Commands and Responses
//-----------------------------------------------------------------------------
#define DNT900CMD_ENTERPROTOCOLMODE			0x00
#define	DNT900CMD_ENTERPROTOCOLMODEREPLY		0x10
#define	DNT900CMD_EXITPROTOCOLMODE			0x01
#define	DNT900CMD_EXITPROTOCOLMODEREPLY			0x11
#define	DNT900CMD_GETREGISTER				0x03
#define	DNT900CMD_GETREGISTERREPLY			0x13
#define	DNT900CMD_SETREGISTER				0x04
#define	DNT900CMD_SETREGISTERREPLY			0x14
#define	DNT900CMD_TXDATA				0x05
#define	DNT900CMD_TXDATAREPLY				0x15
#define	DNT900CMD_RXDATA				0x26

//-----------------------------------------------------------------------------
// DNT900 Registers. The lower byte is the bank and the upper byte is the location
//-----------------------------------------------------------------------------
#define DNT900REG_DEVICEMODE				0x0000
#define DNT900REG_RFDATARATE				0x0100
#define DNT900REG_HOPDURATION				0x0200
#define DNT900REG_INITIALNWKID				0x0200
#define DNT900REG_FREQUENCYBAND				0x0001
#define DNT900REG_ACCESSMODE				0x0101
#define DNT900REG_BASESLOTSIZE				0x0201
#define DNT900REG_REMOTESLOTSIZE			0x0B01
#define DNT900REG_FIRMWAREVERSION			0x0D02
#define DNT900REG_FIRMWAREBUILDNUM			0x0E02
#define DNT900REG_CURRATTEMPTLIMIT			0x1502
#define DNT900REG_SERIALRATE				0x0003


//-----------------------------------------------------------------------------
// DNT900 SerialRate values
//-----------------------------------------------------------------------------
#define DNT900_SERIALRATE_230_4				0x0002

//-----------------------------------------------------------------------------
// DNT900 DeviceMode values
//-----------------------------------------------------------------------------
#define DNT900_DEVICEMODE_REMOTE			0x00
#define DNT900_DEVICEMODE_BASE				0x01
#define DNT900_DEVICEMODE_PTT				0x02

//-----------------------------------------------------------------------------
// DNT900 FrequencyBand values
//-----------------------------------------------------------------------------
#define DNT900_FREQUENCYBAND_NORTHSOUTHAMERICA		0x00
#define DNT900_FREQUENCYBAND_ISRAEL_AUS_NZ		0x01
#define DNT900_FREQUENCYBAND_AUTO			0xFF

//-----------------------------------------------------------------------------
// DNT900 FrequencyBand values
//-----------------------------------------------------------------------------
#define DNT900_FREQUENCYBAND_NORTHSOUTHAMERICA		0x00
#define DNT900_FREQUENCYBAND_ISRAEL_AUS_NZ		0x01
#define DNT900_FREQUENCYBAND_AUTO			0xFF
