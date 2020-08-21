// nRF24lib.h

#ifndef _NRF24LIB_h
#define _NRF24LIB_h

/*-----------------Register Addresses---------------------*/
#define CONFIG 0x00
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define STATUS 0x07
#define RX_PW_P0 0x11
#define RX_CH 0x05
#define RF_SETUP 0x06
#define OBSERVE_TX 0x08
#define Carrier_Detect 0x09
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

#define Pipe0 0x0A
#define Pipe1 0x0B
#define Pipe2 0x0C
#define Pipe3 0x0D
#define Pipe4 0x0E
#define Pipe5 0x0F
#define TX_ADDR 0x10
//---------------------------------------------------------------
//-----------------COMMANDS--------------------------------------
#define W_REGISTER 32  //W_REGISTER + PAYLOAD
#define ACTIVATE 80
#define ACT_DATA 0x73 // to followup with ACTIVATE
#define W_ACK_PAYLOAD 168 //1010 1PPP <--- W_ACK_PAYLOAD + Num of pipes to enable (000 to 101)
#define R_RX_PAYLOAD 97
#define W_TX_PAYLOAD 160
#define FLUX_TX 225
#define FLUX_RX 226

#define MAX_RT 4 // Max retransmission occured
#define TX_DS 5 // set if auto ack received 
#define RX_DR 6 //set when new data arrives RX FIFO
#define RF_DR 3 // Data rate 1 Mbps = 0 / 2Mbps = 1
#define EN_DPL 2 // Enable dynamic payload length
#define EN_ACK_Pay 1 // Enable Acknowledge payload


/*---------------------------------------------------------------*/

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class NRF24libClass
{
 private:
	 byte cePin, csnPin;
	 byte radioChannel, auto_Re_txCount;
	 boolean _dataSent;
	 byte x = 130; // Just some random value does not interact with functionality
	 
	 unsigned long timerTx;
 public:
	 NRF24libClass(byte cePin, byte csnPin);

	void powerOff();
	void rxInit(byte pipeEnable, byte numOfBytes);
	boolean rxOn();
	byte rxReadByte();
	int rxReadInt();
	boolean tx_Ds_Read(); // Data sent interrupt bit read & clear
	void tx_Ds_Clear(); // Data sent interrupt bit read & clear
	boolean rx_Dr_Read(); // Data ready interrupt bit read
	void rx_Dr_Clear(); // Data sent interrupt bit read
	byte readRegister(byte regAddress);
	byte readRegBit(byte regAddress, byte bitNum);
	void printAddrRegister(byte regAddress, byte regWidth);
	void writeRegBit(byte regAddress, byte bitNum, boolean bitVal);//bitNum starting from LSB
	void writeSelectedRegBits(byte regAddress, byte b7, byte b6, byte b5, byte b4, byte b3, byte b2, byte b1, byte b0);
	void writeRegister(byte regAddress, byte regVal);
	void fluxTX();
	void fluxRX();
	void printAllRegVals();
	void rf_ChannelSetup(byte chNum);
	void set_RF_Power(byte powerLevel);
	boolean readCarrierDetect(); // Rx Mode Only
	byte re_Tx_CountRead();
	void auto_Re_TxDelaySet(byte setVal); // 0 to 15
	void auto_Re_TxCountSet(byte setVal); // 0 to 15
	void crc_Settings(boolean crcEnable, boolean crco);// enable = 1 / crco 0 = 1byte, 1 = 2bytes
	void bitRateSetup(boolean bitRate);// 0 or 1
	void writeRegisterBit(byte regAddress, byte bitNum, boolean bitVal);//bitNum starting from LSB
	void activateFeatures();
	void enable_AckPayload();
	void dynSettings();
	void writeAckPayload(int inVal);
	boolean tx_Full();
	boolean rxFull();

	boolean max_RT_Read();
	void max_RT_Clear();
	void txOn(int tx_data, int timeDelay, boolean sPrint); // all in one tx data function 
	void transmitByte(byte payloadIn);
	void transmitInt(int payloadIn);
	byte packetLostCountRead();
	void txInit();

	boolean rxAvailable();
	void addressWidthSetup(byte addressWidth);
	void rx_AddressSetup(byte pipeAddress, uint64_t deviceAddress);
	void tx_AddressSetup(uint64_t deviceAddress);
	
	int rxGet(boolean sPrint);
	void setup_Rx(byte chNum = 22, byte powerLevel = 4, boolean bitRate = 0, boolean crcEnable = 1, boolean crco = 1, byte delaySetVal = 10, byte countSetVal = 15, byte pipeNumbers = B00000001, byte numOfBytes = 2);
	void setup_Tx(byte chNum = 22, byte powerLevel = 4, boolean bitRate = 0, boolean crcEnable = 1, boolean crco = 1, byte delaySetVal = 10, byte countSetVal = 15);
};

//extern NRF24libClass NRF24lib;

#endif

