//-----------------------------------------------------------------------------
// PropManagerRemote.cpp
//
// This file is a base class for all remote property manager instances. It handles the 
// basic functionality of getting information to/from the registers
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

#pragma once

#include "main.h"
#include "PropManagerRemote.h"

using namespace Andromeda;

//-----------------------------------------------------------------------------
// Constructor, initializes all banks to 0
//-----------------------------------------------------------------------------
PropManagerRemote::PropManagerRemote()
{
	_currentBank = 0;
    _currentAddress = 0;
}

//-----------------------------------------------------------------------------
// Sets the value of a particular register. The bank, address and length values
// are required to set the value property
//-----------------------------------------------------------------------------
bool PropManagerRemote::setRegisters(char bank, char address, char length, char *value)
{
	int bankLength;
    char *ptr = 0;
	//get the bank pointer and the total length of the bank
	getBankPointer(bank,ptr,bankLength);

    if( ptr == 0 )
		return false;	

	//make sure we are in bounds
	if( address + length > bankLength )
		return false;

    //copy the value over
    memcpy(ptr + address, value, length); 
}

//-----------------------------------------------------------------------------
// Gets a register at a particular address, length and bank
//-----------------------------------------------------------------------------
bool PropManagerRemote::getRegisters(char bank, char address, char length, char *valueOut)
{
    int bankLength;
    char *ptr = 0;
	//get the bank pointer and the total length of the bank
	getBankPointer(bank,ptr,bankLength);

    if( ptr == 0 )
		return false;	

	//make sure we are in bounds
	if( address + length > bankLength )
		return false;

    //copy the value over
    memcpy(valueOut, ptr + address, length); 
}

//-----------------------------------------------------------------------------
// Gets the property array descriptor which is sent back to a remote module
// wishing to query parameters
//-----------------------------------------------------------------------------
void PropManagerRemote::getPropertyArrayDescriptor(char *buffer, int &lengthOut)
{
	//get total number of banks
	int numBanks = getBankNumber();

    //this consists of the length of each bank in order of bank ID. the first byte contains 
    //the number of banks
    buffer[0] = numBanks;
	for( int i = 0 ; i < numBanks;i++ )
	{
		//get the size of this bank
		int bankLength;
		char *ptr = 0;
		//get the bank pointer and the total length of the bank
		getBankPointer(i,ptr,bankLength);

		//store the bank length
		buffer[i+1] = bankLength;
	}
    lengthOut = numBanks+1;
}

//-----------------------------------------------------------------------------
// Gets the next packet that needs to be downloaded to read the remote property
// manager into the current properties
//-----------------------------------------------------------------------------
void PropManagerRemote::getNextDownloadPacket(char bank, char length)
{

}

//-----------------------------------------------------------------------------
// If a download packet is received, sets the registers held locally based 
// on this data
//-----------------------------------------------------------------------------
void PropManagerRemote::setDownloadPacketRegisters(char bank, char length, char *values)
{

}


//-----------------------------------------------------------------------------
// Internal function used to get the pointer to a bank
//-----------------------------------------------------------------------------
//char * PropManager::getBankPointer(char bank, char address, char length, bool modified /* = false*/)
//{
//    char *ptr = 0;
//    switch(bank)
//    {
//	case 0:
//	//validate the address
//	if( address + length > sizeof(AutopilotRegistersBank0) )
//	return 0;
//	
//	if( modified ) _isBank0Modified = true;
//	ptr = (char *)(&_bank0) + address;
//	return ptr;
//	break;
//	case 1:
//	//validate the address
//	if( address + length > sizeof(AutopilotRegistersBank1) )
//	return 0;
//	
//	if( modified ) _isBank1Modified = true;
//	ptr = (char *)(&_bank1) + address;
//	return ptr;
//	break;
//	
//	case 2:
//	//validate the address
//	if( address + length > sizeof(AutopilotRegistersBank2) )
//	return 0;
//	
//	if( modified ) _isBank2Modified = true;
//	ptr = (char *)(&_bank2) + address;
//	return ptr;
//	break;
//	
//	case 3:
//	//validate the address
//	if( address + length > sizeof(AutopilotRegistersBank3) )
//	return 0;
//	
//	if( modified ) _isBank3Modified = true;
//	ptr = (char *)(&_bank3) + address;
//	return ptr;
//	break;
//	
//	case 4:
//	//validate the address
//	if( address + length > sizeof(AutopilotGainRegistersBank) )
//	return 0;
//	
//	if( modified ) _isBank4Modified = true;
//	ptr = (char *)(&_bank4) + address;
//	return ptr;
//	break;
//	
//	case 5:
//	//validate the address
//	if( address + length > sizeof(AutopilotGainRegistersBank) )
//	return 0;
//	
//	if( modified ) _isBank5Modified = true;
//	ptr = (char *)(&_bank5) + address;
//	return ptr;
//	break;
//	
//	case 6:
//	//validate the address
//	if( address + length > sizeof(AutopilotGainRegistersBank) )
//	return 0;
//	
//	if( modified ) _isBank6Modified = true;
//	ptr = (char *)(&_bank6) + address;
//	return ptr;
//	break;
//	
//	case 7:
//	//validate the address
//	if( address + length > sizeof(AutopilotGainRegistersBank) )
//	return 0;
//	
//	if( modified ) _isBank7Modified = true;
//	ptr = (char *)(&_bank7) + address;
//	return ptr;
//	break;
//	
//	case 8:
//	//validate the address
//	if( address + length > sizeof(AutopilotGainRegistersBank) )
//	return 0;
//	
//	if( modified ) _isBank8Modified = true;
//	ptr = (char *)(&_bank8) + address;
//	return ptr;
//	break;
//	
//	default:
//	return 0;
//    }
//}