
#include "nRF24lib.h"
#include <SPI.h>

NRF24libClass::NRF24libClass(byte cePin_init, byte csnPin_init)
{
	pinMode(cePin_init, OUTPUT);
	pinMode(csnPin_init, OUTPUT);
	cePin = cePin_init;
	csnPin = csnPin_init;
	digitalWrite(csnPin, HIGH);
	timerTx = millis();
}

void NRF24libClass::powerOff()
{
	writeSelectedRegBits(CONFIG, x, x, x, x, x, x, 0, x);
}

void NRF24libClass::rxInit(byte pipeNumbers, byte numOfBytes)
{
	writeSelectedRegBits(CONFIG, x, x, x, x, 1, x, 1, 1); // rx mode on, crc enabled, power up 
	writeRegister(EN_RXADDR, pipeNumbers); // Pipe number to be enabled, B 0000 0001 to B 0011 1111
	writeRegister(RX_PW_P0, numOfBytes); // Num of bytes in selected pipe 
	delay(2);
}

boolean NRF24libClass::rxOn()
{
	digitalWrite(cePin, HIGH);
	boolean bitVal = rx_Dr_Read();
	return bitVal;
}

byte NRF24libClass::rxReadByte()
{
	digitalWrite(cePin, LOW);
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(R_RX_PAYLOAD);
	byte dummybyte1 = SPI.transfer(0xFF);
	digitalWrite(csnPin, HIGH);
	rx_Dr_Clear();

	return dummybyte1;
}

int NRF24libClass::rxReadInt()
{
	digitalWrite(cePin, LOW);
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(R_RX_PAYLOAD);
	int dummybyte1 = SPI.transfer16(0xFF);
	digitalWrite(csnPin, HIGH);
	rx_Dr_Clear();

	return dummybyte1;
}

boolean NRF24libClass::tx_Ds_Read() // Data sent interrupt bit read & clear
{
	boolean bitVal = readRegBit(STATUS, TX_DS);
	return bitVal;
}

void NRF24libClass::tx_Ds_Clear() // Data sent interrupt bit read & clear
{
	writeSelectedRegBits(STATUS, x, 0, 1, x, x, x, x, x);
}

boolean NRF24libClass::rx_Dr_Read() // Data ready interrupt bit read
{
	boolean bitVal = readRegBit(STATUS, RX_DR);
	return bitVal;
}

void NRF24libClass::rx_Dr_Clear() // Data sent interrupt bit read
{
	writeSelectedRegBits(STATUS, x, 1, 0, x, x, x, x, x);
}


byte NRF24libClass::readRegister(byte regAddress)
{
	digitalWrite(csnPin, LOW);
	byte statusVal = SPI.transfer(regAddress);
	byte inbyte = SPI.transfer(0);
	digitalWrite(csnPin, HIGH);
	return  inbyte;
}

byte NRF24libClass::readRegBit(byte regAddress, byte bitNum)
{
	digitalWrite(csnPin, LOW);
	byte statusVal = SPI.transfer(regAddress);
	byte inbyte = SPI.transfer(0);
	digitalWrite(csnPin, HIGH);
	return bitRead(inbyte, bitNum);
}

void NRF24libClass::printAddrRegister(byte regAddress, byte regWidth)
{
	byte inbyte[6];
	digitalWrite(csnPin, LOW);
	byte statusVal = SPI.transfer(regAddress);
	for (int i = 0; i < regWidth; i++)
	{
		inbyte[i] = SPI.transfer(0);
		Serial.println(inbyte[i], HEX);
	}
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::writeRegBit(byte regAddress, byte bitNum, boolean bitVal)//bitNum starting from LSB
{
	byte regVal = readRegister(regAddress);
	if (bitVal)
	{
		bitSet(regVal, bitNum);
	}
	else
	{
		bitClear(regVal, bitNum);
	}
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(32 + regAddress);
	byte dummybyte1 = SPI.transfer(regVal);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::writeSelectedRegBits(byte regAddress, byte b7, byte b6, byte b5, byte b4, byte b3, byte b2, byte b1, byte b0)
{// Assign Set(1) or clear(0) bits or assign "x" to bits that do not want to change
	byte regVal = readRegister(regAddress);
	byte b[8] = { b0, b1, b2, b3, b4, b5, b6, b7 };
	for (byte i = 0; i < 8; i++)
	{
		if (b[i] == 1)
		{
			bitSet(regVal, i);
		}
		else if (b[i] == 0)
		{
			bitClear(regVal, i);
		}
	}
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(32 + regAddress);
	byte dummybyte1 = SPI.transfer(regVal);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::writeRegister(byte regAddress, byte regVal)
{
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(32 + regAddress);
	byte dummybyte1 = SPI.transfer(regVal);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::fluxTX()
{
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(FLUX_TX);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::fluxRX()
{
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(FLUX_RX);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::printAllRegVals()
{
	for (int i = 0; i < 24; i++)
	{
		Serial.println(readRegister(i), BIN);
		delay(10);
	}
}

void NRF24libClass::rf_ChannelSetup(byte chNum)
{
	radioChannel = chNum;
	writeRegister(RF_CH, chNum);
}

void NRF24libClass::set_RF_Power(byte powerLevel)
{
	byte regVal = B11111001 & readRegister(RF_SETUP);
	byte outVal = regVal | powerLevel << 1;
	writeRegister(RF_SETUP, outVal);
}

boolean NRF24libClass::readCarrierDetect() // Rx Mode Only
{
	return readRegBit(Carrier_Detect, 0);
}

byte NRF24libClass::re_Tx_CountRead()
{
	byte inbyte = B00001111 & readRegister(OBSERVE_TX);
	return inbyte;
}

void NRF24libClass::auto_Re_TxDelaySet(byte setVal) // 0 to 15
{
	byte inbyte = B00001111 & readRegister(SETUP_RETR);
	byte regVal = inbyte | setVal << 4;

	writeRegister(SETUP_RETR, regVal);
}

void NRF24libClass::auto_Re_TxCountSet(byte setVal) // 0 to 15
{
	auto_Re_txCount = setVal;
	byte inbyte = B11110000 & readRegister(SETUP_RETR);
	byte regVal = inbyte | setVal;

	writeRegister(SETUP_RETR, regVal);
}

void NRF24libClass::crc_Settings(boolean crcEnable, boolean crco)// enable = 1 / crco: 0 = 1byte, 1 = 2bytes
{
	byte inbyte = B11110011 & readRegister(CONFIG);
	byte outbyte = inbyte | crcEnable << 3 | crco << 2;
	writeRegister(CONFIG, outbyte);
}

void NRF24libClass::bitRateSetup(boolean bitRate)// 0 or 1
{
	writeRegisterBit(RF_SETUP, 3, bitRate);
}

void NRF24libClass::writeRegisterBit(byte regAddress, byte bitNum, boolean bitVal)//bitNum starting from LSB
{
	byte regVal = readRegister(regAddress);
	if (bitVal)
	{
		bitSet(regVal, bitNum);
	}
	else
	{
		bitClear(regVal, bitNum);
	}
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(32 + regAddress);
	byte dummybyte1 = SPI.transfer(regVal);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::activateFeatures()
{
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(ACTIVATE);
	byte dummybyte1 = SPI.transfer(ACT_DATA);
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::enable_AckPayload()
{
	writeRegister(FEATURE, B00000110);
}

void NRF24libClass::dynSettings()
{
	activateFeatures();
	enable_AckPayload();
	writeRegister(DYNPD, 1);
}

void NRF24libClass::writeAckPayload(int inVal)
{
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(W_ACK_PAYLOAD);
	int dummybyte1 = SPI.transfer16(inVal);
	digitalWrite(csnPin, HIGH);
}

boolean NRF24libClass::tx_Full()
{
	return readRegBit(FIFO_STATUS, 5);
}

boolean NRF24libClass::rxFull()
{
	return readRegBit(FIFO_STATUS, 1);
}

boolean NRF24libClass::max_RT_Read()
{
	boolean bitVal = readRegBit(STATUS, MAX_RT);
	return bitVal;
}

void NRF24libClass::max_RT_Clear()
{
	writeRegBit(STATUS, MAX_RT, 1);
}

void NRF24libClass::txOn(int tx_data, int timeDelay, boolean sPrint)  // all in one tx data function 
{
	if (millis() - timerTx > timeDelay)
	{
		//FluxTX();
		//TransmitByte(23);
		transmitInt(tx_data);
		_dataSent = 1;
		timerTx = millis();
	}
	
	/*if (RX_DR_Read())
	{
	int rxReceived = RxReadInt();

	Serial.print(" received= ");
	Serial.println(rxReceived);
	RX_DR_Clear();
	}*/
	
	if(_dataSent)
	{
		if (max_RT_Read())
		{
			max_RT_Clear();
		}
		
		if (tx_Ds_Read())
		{
			if(sPrint)
			{
				Serial.println(" send successful");
			}
			tx_Ds_Clear();
			_dataSent = 0;		
		}	
		
		if (packetLostCountRead() == auto_Re_txCount) // <-- Need to be revised
		{
				rf_ChannelSetup(radioChannel); // depends on rf channel <----------
				fluxTX();
		}		
	}	
}

int NRF24libClass::rxGet(boolean sPrint) //<------------------------All in one Rx data function
{
	if (rxAvailable())
	{
		int rxReceived = rxReadInt();
		
		if(sPrint)
		{
			Serial.print(" received = ");
			Serial.println(rxReceived);
		}
		return rxReceived;
	}
}

void NRF24libClass::transmitByte(byte payloadIn)
{
digitalWrite(csnPin, LOW);
byte dummybyte0 = SPI.transfer(W_TX_PAYLOAD);
byte dummybyte1 = SPI.transfer(payloadIn);

digitalWrite(csnPin, HIGH);
digitalWrite(cePin, HIGH);
delayMicroseconds(20);
digitalWrite(cePin, LOW);
}

void NRF24libClass::transmitInt(int payloadIn)
{
	digitalWrite(csnPin, LOW);
	byte dummybyte0 = SPI.transfer(W_TX_PAYLOAD);
	int dummybyte1 = SPI.transfer16(payloadIn);
	digitalWrite(csnPin, HIGH);

	digitalWrite(cePin, HIGH);
	delayMicroseconds(100);
	digitalWrite(cePin, LOW);
}

byte NRF24libClass::packetLostCountRead()
{
	byte inbyte = readRegister(OBSERVE_TX);
	return inbyte >> 4;
}

void NRF24libClass::txInit() // Enable Tx mode
{
	writeSelectedRegBits(CONFIG, x, x, x, x, x, x, 1, 0);
	delay(2);
}

boolean NRF24libClass::rxAvailable()
{
	digitalWrite(cePin, HIGH);
	boolean bitVal = rx_Dr_Read();
	return bitVal;
}

void NRF24libClass::addressWidthSetup(byte addressWidth) // 3, 4 or 5 (bytes)
{
	if (addressWidth == 3)
	{
		writeRegister(SETUP_AW, 1);
	}
	else if(addressWidth == 4)
	{
		writeRegister(SETUP_AW, 2);
	}
	else 
	{
		writeRegister(SETUP_AW, 3);
	}
}

void NRF24libClass::rx_AddressSetup(byte regAddress, uint64_t deviceAddress) 
{//address = 0x00 to 0xFF FF FF FF FF (5 bytes) for data pipe0 and pipe1
	// All other 4 pipes has 1 LSByte to write 
	digitalWrite(csnPin, LOW);
	byte statusVal = SPI.transfer(32 + regAddress);

	if (regAddress == 0x0A || regAddress == 0x0B || regAddress == 0x10)
	{
		for (int i = 0; i < 5; i++)
		{
			SPI.transfer(deviceAddress >> 8 * i);
		}
	}
	else
	{
		SPI.transfer(deviceAddress);
	}	
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::tx_AddressSetup(uint64_t deviceAddress)
{//address = 0x00 to 0xFF FF FF FF FF (5 bytes) 
	digitalWrite(csnPin, LOW);
	byte statusVal = SPI.transfer(32 + 0x10);

	for (int i = 0; i < 5; i++)
	{
		SPI.transfer(deviceAddress >> 8 * i);
	}
	
	digitalWrite(csnPin, HIGH);
}

void NRF24libClass::setup_Rx(byte chNum, byte powerLevel, boolean bitRate, boolean crcEnable, boolean crco, byte delaySetVal, byte countSetVal, byte pipeNumbers, byte numOfBytes)
{
	rf_ChannelSetup(chNum);
	//-------------------------------------

	set_RF_Power(powerLevel); // 0 to 4
	bitRateSetup(bitRate); // 0 or 1 = 1mbps or 2mbps
	crc_Settings(crcEnable, crco); // enable = 1 / crco: 0 = 1byte, 1 = 2bytes
	auto_Re_TxDelaySet(delaySetVal); // 0 to 15
	auto_Re_TxCountSet(countSetVal); // 0 to 15


	rxInit(pipeNumbers, numOfBytes); // Enable pipe

	//Serial.println(readRegister(RX_CH), BIN);
	//Serial.println(readRegister(FEATURE), BIN);
	//Serial.println(readRegister(DYNPD), BIN);
}

void NRF24libClass::setup_Tx(byte chNum, byte powerLevel, boolean bitRate, boolean crcEnable, boolean crco, byte delaySetVal, byte countSetVal)
{
	rf_ChannelSetup(chNum);
	//-------------------------------------

	set_RF_Power(powerLevel); // 0 to 4
	bitRateSetup(bitRate); // 0 or 1 = 1mbps or 2mbps
	crc_Settings(crcEnable, crco); // enable = 1 / crco: 0 = 1byte, 1 = 2bytes
	auto_Re_TxDelaySet(delaySetVal); // 0 to 15
	auto_Re_TxCountSet(countSetVal); // 0 to 15

	txInit(); // Enable Tx mode
}

//NRF24libClass NRF24lib;

