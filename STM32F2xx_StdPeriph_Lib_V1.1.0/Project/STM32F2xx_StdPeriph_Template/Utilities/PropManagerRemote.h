//-----------------------------------------------------------------------------
// PropManagerRemote.h
//
// This file is a base class for all remote property manager instances. It handles the 
// basic functionality of getting information to/from the registers
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#pragma once

#define PROP_FLAG_MODIFIED 1	//flag which specifies whether or not the value has been modified
#define PROP_FLAG_DOWNLOADED 2 //flag which specifies whether the value has been initially downloaded

namespace Andromeda
{
  class PropManagerRemote
  {
	private:
	int _currentBank;		//the current bank that needs to be downloaded
	int _currentAddress;   //the current address that needs to be downloaded

	protected:
	virtual bool getBankPointer(int bankNum, char *&pointer, int &length) = 0;
	virtual int getBankNumber() = 0;
    
    public: 
    PropManagerRemote();
    bool setRegisters(char bank, char address, char length, char *value);
    bool getRegisters(char bank, char address, char length, char *valueOut);

    void getPropertyArrayDescriptor(char *buffer, int &lengthOut);
    virtual void setBankFlags(int bankNum, char flag) = 0;
	virtual char getBankFlags(int bankNum) = 0;

	//mass download functions
	void getNextDownloadPacket(char bank, char length);
	void setDownloadPacketRegisters(char bank, char length, char *values);
  };
	
  

}
