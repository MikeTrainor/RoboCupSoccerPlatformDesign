/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "MKL25Z4.h"

#define CE_HIGH()	GPIOE_PSOR |= 0x01 ;
#define CE_LOW()	GPIOE_PCOR |= 0x01;

#include "stddef.h"

 ////////////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////////////
 //SPI
#define SPI_PUSHR_PCS0_ON 0x10000
#define SPI_CTAR_FMSZ_8BIT 0x38000000
//RF
#define PACKETSIZE 7
#define RECEIVEPACKETSIZE 8
////////////////////////////////////////////////////////////////////////////////////////////
/*****										Instructions										*****/
/// Read Register
#define R_REGISTER    0x00
/// Write Register
#define W_REGISTER    0x20
/// Read receive payload (clears FIFO; LSByte first)
#define R_RX_PAYLOAD  0x61
/// Write transmit payload
#define W_TX_PAYLOAD  0xa0
/// Flush transmit FIFO
#define FLUSH_TX      0xe1
/// Flush receive FIFO (should not be used while an ack is being transmitted)
#define FLUSH_RX      0xe2
/// Reuse transmit payload.  Use this to continuously retransmit the payload (in Tx mode) as long as CE is held high, until
/// the Tx FIFO is flushed or the payload is overwritten.  This should not be changed while transmitting.  This is different
/// from Enhanced Shockburst.
#define REUSE_TX_PL   0xe3
/// No operation.
#define NOP           0xff


#define REGISTER_MASK 0x1F
#define CONFIG      0x00
/// Enhanced Shockburst (auto-ack) on the Rx pipes.
#define EN_AA       0x01
/// Enable or disable the Rx pipes
#define EN_RXADDR   0x02
/// Set address width (must be the same on all radios)
#define SETUP_AW    0x03
/// Set retry delay and number of retries for pipes using Enhanced Shockburst
#define SETUP_RETR  0x04
/// Set the channel to use, from the 2.4 GHz ISM band.
#define RF_CH       0x05
/// Setup radio data rate, output power, and LNA gain
#define RF_SETUP    0x06
/// Interrupt status, Tx FIFO full, and number of the Rx pipe that received a packet.
#define STATUS      0x07
/// Count lost and resent packets
#define OBSERVE_TX  0x08
/// Carrier detect (LSB is 1 if the receiver detects an in-band signal for at least 128 us)
#define CD          0x09
/// Receive address bytes (see documentation for explanation of how they fit 6 5-byte addresses
/// into 40 bits).  P0 is also used for auto-ack handling.
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
/// Transmit destination address.
#define TX_ADDR     0x10
/// Payload data width for each of the Rx pipes (0x01 bytes to 0x20 bytes, or 0x00 if pipe is not used)
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
/// Auto-retransmit status (cf. REUSE_TX_PL instruction), Tx FIFO full/empty, Rx FIFO full/empty
/// (The Rx FIFO is a 3-packet queue shared by all six pipes)
#define FIFO_STATUS 0x17

// definitions
char RADIO_PIPE_0 = 0;
char RADIO_PIPE_1 = 1;
char RADIO_PIPE_2 = 2;
char RADIO_PIPE_3 = 3;
char RADIO_PIPE_4 = 4;
char RADIO_PIPE_5 = 5;
char RADIO_PIPE_EMPTY = 7;

char  rx_pipe_widths[6] = { 32, 32, 0, 0, 0, 0 };
char  rx_pipe0_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };
static  uint8_t tx_address[5] = { 0xe7, 0xe7, 0xe7, 0xe7, 0xe7 };

#define ADDRESS_LENGTH 5
#define CHANNEL 112


/// RF_SETUP Register
/// Force PLL lock signal.  See http://en.wikipedia.org/wiki/Phase-locked_loop.  This shouldn't be set.
#define PLL_LOCK    4
/// 0 - On-air data rate is 1 Mbps.  1 - On-air data rate is 2 Mbps.
#define RF_DR       3
/// Radio Tx Power (bits 2:1)
/// 00 - -18 dBm.  01 - -12 dBm.  10 - -6 dBm.  11 - 0 dBm.  dBm is decibels relative to 1 mW.
#define RF_PWR      1
/// Low-noise amplifier gain.  This shouldn't be cleared.
#define LNA_HCURR   0

/// CONFIG Register
/// 0 - Rx data-ready interrupt is sent to IRQ pin.  1 - Interrupt is not sent to IRQ pin.
#define MASK_RX_DR  6
/// 0 - Tx data-sent interrupt is sent to IRQ pin.  1 - Interrupt is not sent to IRQ pin.
#define MASK_TX_DS  5
/// 0 - Max-retries-reached interrupt is sent to IRQ pin.  1 - Interrupt is not sent to IRQ pin.
#define MASK_MAX_RT 4
/// 0 - Disable automatic CRC.  1 - Enable automatic CRC.
#define EN_CRC      3
/// 0 - Use 1-byte CRC.  1 - Use 2-byte CRC.
#define CRCO        2
/// 0 - Power down the radio.  1 - Power up the radio
#define PWR_UP      1
/// 0 - Radio is a transmitter.  1 - Radio is a receiver.
#define PRIM_RX     0

/// STATUS Register
/// 0 - Rx data ready interrupt was not triggered.  1 - Rx data ready interrupt was triggered.  (write 1 to clear after interrupt)
#define RX_DR       6
/// 0 - Tx data sent interrupt was not triggered.  1 - Tx data sent interrupt was triggered.  (write 1 to clear after interrupt)
/// If Enhanced Shockburst is enabled, the interrupt is triggered once the transmitter receives the ack.
#define TX_DS       5
/// 0 - Max retries sent interrupt was not triggered.  1 - Max retries sent interrupt was triggered.  (write 1 to clear after interrupt)
/// If the MAX_RT interrupt is triggered, this needs to be cleared before the radio can be used again.
#define MAX_RT      4
/// Number of the data pipe that just received data (bits 3:1)
/// 000 to 101 - pipe number 0-5.  110 not used.  111 all FIFOs are empty.
#define RX_P_NO     1
/// 0 - There are available locations in the Tx FIFO.  1 - The Tx FIFO is full.
#define TX_FULL     0

/// OBSERVE_TX Register
/// Lost packet count (bits 7:4)
/// Counts up to 15 lost packets (does not overflow).  Reset by writing to RF_CH.
#define PLOS_CNT    4
/// Resent packet count (bits 3:0)
/// Counts the packets that were re-sent in Enhanced Shockburst mode.  Reset by sending a new packet.
#define ARC_CNT     0

char  transmit_lock;

char RADIO_LOWEST_POWER = 0;		// -18 dBm (about 16 uW)
char RADIO_LOW_POWER = 1;		// -12 dBm (about 63 uW)
char RADIO_HIGH_POWER = 2;		// -6 dBm (about 251 uW)
char RADIO_HIGHEST_POWER = 3;	// 0 dBm (1 mW)


char RADIO_1MBPS = 0;		// that's Mbps, not MBps.
char RADIO_2MBPS = 1;

char tx_last_status = 1;  // successfuly transmitted data when 1 and 0, otherwise.
char tx_history = 0xFF;		// Used to find the radio success rate

char RADIO_TX_SUCCESS = 1; // data transmit was succesful
char RADIO_TX_MAX_RT = 0; // max amount of retiried attempts was reached



#define RADIO_RX_INVALID_ARGS	0x01		// one of the arguments to Radio_Receive was invalid
#define RADIO_RX_TRANSMITTING	0x02	// the radio was transmitting
#define RADIO_RX_FIFO_EMPTY		0x03 // there aren't any packets in the Rx FIFO to receive (Radio_Receive does not receive data)
#define RADIO_RX_MORE_PACKETS	0x04	// after copying out the head of the Rx FIFO, there is still another packet in the FIFO.
#define RADIO_RX_SUCCESS		0x05


char received_data[RECEIVEPACKETSIZE];

// Transmit to UART variables for real time plot generation
char IDr;
char Speed1r, Speed2r, Speed3r, Speed4r;


void SCGC_CLOCKS() {
	// SCGC4
	SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK; // SPI0 MASK
	// SCGC5
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

}

void GPIO_Init(){

	// MULTIPLEXER:
			// RF-Module----ON BOARD SPI0
	PORTD_PCR0 |= PORT_PCR_MUX(2); // CSN-----------PCS0
	PORTD_PCR1 |= PORT_PCR_MUX(2); // SCK------------SCK
	PORTD_PCR2 |= PORT_PCR_MUX(2); // MOSI----------MOSI
	PORTD_PCR3 |= PORT_PCR_MUX(2); // MISO----------MISO
	PORTE_PCR0 |= PORT_PCR_MUX(1); // CE--------------CE
	PORTD_PCR5 |= PORT_PCR_MUX(1); // IRQ-----------PTD5

	// Interrupts:
			// Set interrupt for the IRQ pin of the NRF24L01
	PORTD_PCR5 &= ~(PORT_PCR_ISF_MASK);
	PORTD_PCR5 |= PORT_PCR_IRQC(7); // set the IRQC flag to detect the falling edge of the // IDK IF THIS IS CORRECT

			// Enable interrupt and clear status flag
	PORTD_ISFR = PORT_ISFR_ISF(0x05); // 0x01 = 0b1 this selects PTD5 for clear interrupt status flag
	NVIC_EnableIRQ(PORTD_IRQn); // This Enables the interrupt

	// PDDR:
	GPIOE_PDDR |= 0x01; // Set CE as output
	GPIOD_PDDR &= ~(0x01 << 5); // Set IRQ as INPUT
}


void SPI_INIT() {
	SPI0_C1 |= SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK | SPI_C1_CPHA_MASK | SPI_C1_SSOE_MASK;
	SPI0_C2 |= SPI_C2_MODFEN_MASK;
	SPI0_BR |= 0x01 << 5; // We are shifting by 5 to get a baud rate of 12.5 MHz
	// Note- Data sheet specifies that Max baud rate is 10 MHz however acording to many people
	// who tested out this module, they can run up to 16~18 MHz no problem
}


void SPI_READ_WRITE(uint8_t* data, char* buffer, char len) {

	char i = 0;
	for (i; i < len; i++) {
		while(!(SPI0_S & SPI_S_SPTEF_MASK));
		SPI0_D = data[i];

		while(!(SPI0_S & SPI_S_SPRF_MASK) )
		buffer[i] = SPI0_D;

	}
}


void SPI_Write_Block(uint8_t* data, char len){
    char i;
    for (i = 0; i < len; i++) {
        while(!(SPI0_S & SPI_S_SPTEF_MASK));
		SPI0_D = data[i];
    }
}


char SPI_WRITE_BYTE(uint8_t byte) {

	char data;

	while(!(SPI0_S & SPI_S_SPTEF_MASK));
	SPI0_D = byte;

	while(!(SPI0_S & SPI_S_SPRF_MASK) )
	data = SPI0_D;

	return data;
}


// NRF24L01 functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
char get_status() {
	char status = 0;
	status = SPI_WRITE_BYTE(NOP);
	return status;
}

char set_register(char reg, char* value, char len) {
	char status;

	status = SPI_WRITE_BYTE(W_REGISTER | (REGISTER_MASK & reg));
	SPI_Write_Block(value, len);

	return status;
}

char get_register(char reg, char* buffer, char len) {
	char status;
	char i;
	for (i = 0; i < len; i++) {
		buffer[i] = 0xFF;
	}

	status = SPI_WRITE_BYTE(R_REGISTER | (REGISTER_MASK & reg));
	SPI_READ_WRITE(NULL, buffer, len);

	return status;
}

void send_instruction(char instruction, char* data, char* buffer, char len) {
	SPI_WRITE_BYTE(instruction);

	if (len > 0) {
		if (buffer == NULL)
			SPI_Write_Block(data, len);
		else
			SPI_READ_WRITE(data, buffer, len);
	}
}

void set_rx_mode() {
	char config;

	get_register(CONFIG, &config, 1);
	if ((config & PRIM_RX) == 0) {
		config |= PRIM_RX;
		set_register(CONFIG, &config, 1);

		/* INSERT DELAY FUNCTION ----- 130 uSeconds ------*/
		int i,j;
		for (i=0; i< 20000; i++){};
	}
}

void set_tx_mode() {
	char config;
	int i;
	get_register(CONFIG, &config, 1);
	if ((config & PRIM_RX) != 0) {
		config &= ~PRIM_RX;
		set_register(CONFIG, &config, 1);

		/* INSERT DELAY FUNCTION ----- 130 uSeconds ------*/
		for (i=0; i< 20000; i++){};
	}
}

void reset_pipe0_address() {
	if (rx_pipe_widths[RADIO_PIPE_0] != 0) {
		set_register(RX_ADDR_P0, (char*)rx_pipe0_address, ADDRESS_LENGTH);
	}
}


void Radio_Init() {

	transmit_lock = 0;
	CE_LOW();
	int i,j;

	/*  --------- INSTER 11 mSECONDS DELAY HERE ---------- */
		for (i=0; i< 100000; i++){};

	configure_registers();

	/*  --------- INSTER 2 mSECONDS DELAY HERE ---------- */
		for (i=0; i< 20000; i++){};

	CE_HIGH();
}

void configure_registers() {
	char value;
	SPI_INIT();

	// set address width to 5 bytes.
	value = ADDRESS_LENGTH - 2;			// 0b11 for 5 bytes, 0b10 for 4 bytes, 0b01 for 3 bytes
	set_register(SETUP_AW, &value, 1);
	// set Enhanced Shockburst retry to every 586 us, up to 5 times.  If packet collisions are a problem even with AA enabled,
	// then consider changing the retry delay to be different on the different stations so that they do not keep colliding on each retry.
	value = 0x15;
	//value = 0x10;
	set_register(SETUP_RETR, &value, 1);

	// Set to use 2.4 GHz channel 110.
	value = CHANNEL;
	set_register(RF_CH, &value, 1);

	// Set radio to 2 Mbps and high power.  Leave LNA_HCURR at its default.
	value = RF_DR | LNA_HCURR;
	set_register(RF_SETUP, &value, 1);

	// Enable 2-byte CRC and power up in receive mode.
	value = EN_CRC | CRCO | PWR_UP | PRIM_RX;
	set_register(CONFIG, &value, 1);

	// clear the interrupt flags in case the radio's still asserting an old unhandled interrupt
	value = RX_DR | TX_DS | MAX_RT;
	set_register(STATUS, &value, 1);



	// flush the FIFOs in case there are old data in them.
	send_instruction(FLUSH_TX, NULL, NULL, 0);
	send_instruction(FLUSH_RX, NULL, NULL, 0);
}

void Radio_Configure_Rx(char pipe, char* address, char enable)
{
	char value;
	char use_aa = 1;
	char payload_width = 32;

	if (payload_width < 1 || payload_width > 32 || pipe < RADIO_PIPE_0 || pipe > RADIO_PIPE_5) return;

	// store the pipe 0 address so that it can be overwritten when transmitting with auto-ack enabled.
	if (pipe == RADIO_PIPE_0)
	{
		rx_pipe0_address[0] = address[0];
		rx_pipe0_address[1] = address[1];
		rx_pipe0_address[2] = address[2];
		rx_pipe0_address[3] = address[3];
		rx_pipe0_address[4] = address[4];
	}

	set_register(RX_ADDR_P0 + pipe, address, pipe > RADIO_PIPE_1 ? 1 : ADDRESS_LENGTH);

	// Set auto-ack.
	get_register(EN_AA, &value, 1);
	if (use_aa)
		value |= pipe;
	else
		value &= ~pipe;
	set_register(EN_AA, &value, 1);

	value = enable ? payload_width : 0;
	set_register(RX_PW_P0 + pipe, &value, 1);
	rx_pipe_widths[pipe] = value;

	// Enable or disable the pipe.
	get_register(EN_RXADDR, &value, 1);
	if (enable)
		value |= pipe;
	else
		value &= ~pipe;
	set_register(EN_RXADDR, &value, 1);

}

void Radio_Set_Tx_Addr(char* address) {
	tx_address[0] = address[0];
	tx_address[1] = address[1];
	tx_address[2] = address[2];
	tx_address[3] = address[3];
	tx_address[4] = address[4];
	set_register(TX_ADDR, address, ADDRESS_LENGTH);
}

void Radio_Configure(char dr, char power) {

	char value;
	if (power < RADIO_LOWEST_POWER || power > RADIO_HIGHEST_POWER || dr < RADIO_1MBPS || dr > RADIO_2MBPS) return;

	Radio_Set_Tx_Addr(tx_address);

	get_register(RF_SETUP, &value, 1);

	value |= 3 << RF_PWR;			// set the power bits so that the & will mask the power value in properly.
	value &= power << RF_PWR;		// mask the power value into the RF status byte.

	if (dr)
		value |= RF_DR;
	else
		value &= ~RF_DR;

	set_register(RF_SETUP, &value, 1);
}
////////////////////////////////////////////////////////////////////////////////////////////
// here we have the acctual transmit and receive functions--- An ISR Code for the IRQ PIN of the NRF24l01
// and the PTC18 pin of the FRDMK64F12 must be implemented to update the transmit_lock variables and other stuff

void Radio_Transmit(char* payload, char wait) {

	char len = PACKETSIZE;
	transmit_lock = 1;

	CE_LOW();

	set_tx_mode();

	set_register(RX_ADDR_P0, (char*)tx_address, ADDRESS_LENGTH);

	send_instruction(W_TX_PAYLOAD, payload, NULL, len);

	CE_HIGH();

	if (wait == 1) {
		while (transmit_lock);
		return tx_last_status;
	}

	return 1;
}


char Radio_Recieve(char* buffer) {
	char len = 32;
	char status;
	char pipe_number;
	char doMove = 1;
	char result;

	transmit_lock = 0;

	CE_LOW();

	status = get_status();
	pipe_number = (status & 0x0E) >> 1;

	if (pipe_number == RADIO_PIPE_EMPTY)
	{
		result = RADIO_RX_FIFO_EMPTY;
		doMove = 0;
	}

	if (rx_pipe_widths[pipe_number] > len)
	{
		// the buffer isn't big enough, so don't copy the data.
		result = RADIO_RX_INVALID_ARGS;
		doMove = 0;
	}

	if (doMove)
	{
		// Move the data payload into the local
		send_instruction(R_RX_PAYLOAD, (uint8_t*)buffer, (uint8_t*)buffer, rx_pipe_widths[pipe_number]);

		status = get_status();
		pipe_number = (status & 0xE) >> 1;

		if (pipe_number != RADIO_PIPE_EMPTY)
			result = RADIO_RX_MORE_PACKETS;
		else
			result = RADIO_RX_SUCCESS;
	}

	CE_HIGH();

	transmit_lock = 0;



	//release_radio();

	return result;
}

// IDK IF WE NEED THIS
char Radio_Success_Rate() {
	char wh = tx_history;
	char weight = 0;

	while (wh != 0) {
		if ((wh & 1) != 0) weight++;
		wh >>= 1;
	}
	wh = (16 - weight) * 100;
	wh /= 16;
	return wh;

}

void Radio_Flush()
{
	send_instruction(FLUSH_TX, NULL, NULL, 0);
	send_instruction(FLUSH_RX, NULL, NULL, 0);
}





#define NRF_IRQ 0x05


void RF_Interrupt(){
	char status;
	char pipe_number;


	CE_LOW();

	status = get_status();

	if (status & RX_DR)
	{
		pipe_number = (status & 0xE) >> 1;
		//radio_rxhandler(pipe_number);  // WTF IS THIS AND WHY ARE WE NOT USING RADIO_RECIEVE ???


		Radio_Recieve(received_data); // I THINK THIS WOULD BE THE THING WE SHOULD ACCTUALLY DO??

		/* Add this when the code is working
		IDr= received_data[1];
		Speed1r = received_data[2];
		Speed2r = received_data[3];
		Speed3r = received_data[4];
		Speed4r = received_data[5];

		// Send Sensor data to Master Computer
		Resp_to_Mstr(IDr, Spee1r, Speed2r, Speed3r, Speed4r);
			*/


	}
	// We can get the TX_DS or the MAX_RT interrupt, but not both.
	if (status & TX_DS)
	{
		// if there's nothing left to transmit, switch back to receive mode.
		transmit_lock = 0;
		reset_pipe0_address();
		set_rx_mode();

		// indicate in the history that a packet was transmitted successfully by appending a 1.
		tx_history <<= 1;
		tx_history |= 1;

		tx_last_status = RADIO_TX_SUCCESS;
	}
	else if (status & MAX_RT)
	{
		send_instruction(FLUSH_TX, NULL, NULL, 0);

		transmit_lock = 0;
		reset_pipe0_address();
		set_rx_mode();
		// indicate in the history that a packet was dropped by appending a 0.
		tx_history <<= 1;

		tx_last_status = RADIO_TX_MAX_RT;
	}

	// clear the interrupt flags.
	status = RX_DR | TX_DS | MAX_RT;
	set_register(STATUS, &status, 1);

	CE_HIGH();
}





void PORTD_IRQHandler(void){
	char PTD_staus = GPIOD_PDIR;
	RF_Interrupt();
	PORTD_PCR5 |= PORT_PCR_ISF_MASK; //Reset Interrupt Status Flag
}


void SendPacket(char ID_bit, char MTR_Speed1, char MTR_Speed2, char dir1, char dir2) {
	char transmit[PACKETSIZE];
	char Start_bit = 0xFF;
	char End_bit = 0xFF;

	transmit[0] = Start_bit;
	transmit[1] = ID_bit;
	transmit[2] = MTR_Speed1;
	transmit[3] = MTR_Speed2;
	transmit[4] = dir1;
	transmit[5] = dir2;
	transmit[7] = End_bit;


	Radio_Transmit(transmit, 1);
}

void main() {
	// Transmit to SPI variables
	char ID;
	char Speed1, dir1, Speed2, dir2;

	// Initialization
	SCGC_CLOCKS();

	GPIO_Init();

	// Fill this out
	Radio_Init();
	Radio_Configure(1, 0x11); // 1-> AIR DATA RATE. 0x11-> POWER -- (make sure 0x11 is set to MAX POWER) --
	Radio_Configure_Rx(RADIO_PIPE_0, rx_pipe0_address, 1); // 1-> enables the pipe


	ID = 5;
	Speed1 = 100;
	Speed2 = 100;



	while (1) {

		SendPacket(ID, Speed1, Speed2, dir1, dir2);// add some stuff

	}

}
