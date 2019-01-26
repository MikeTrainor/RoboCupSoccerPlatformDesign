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
 *   software without velecific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, velECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MKL25Z4.h"

// Pin Assignments Page 161/807
// PWM Module Page 547/807
// GPIO Registers Page 776/807
// http://makeatronics.blogvelot.com/2013/02/efficiently-reading-quadrature-with.html
// https://www.dialog-semiconductor.com/sites/default/files/an-cm-250_position_and_speed_control_of_a_dc_motor_using_analog_pid_controller.pdf

/*PORTS IN USE
 * PTC1, PTC2
 * PTC3, PTC4
 * PTA5, PTC9 = PWM Output
 *
 */

// Global Variables
unsigned int encoder1_data1, encoder1_data2, encoder2_data1, encoder2_data2;
unsigned int des_vel, mes_vel, des_vel2, mes_vel2;
unsigned int e, e2, e_Prev2 = 0;
unsigned int P_Term, I_Term, D_Term, P_Term2, I_Term2, D_Term2;
unsigned int I_Term_Min = 0, I_Term_Max = 5, e_Prev = 0;
unsigned int velmax, ymax, y, ydot, ydotmax, umax = 0x0A;
unsigned int speed_motor0 = 0, speed_motor1 = 0;
unsigned int Kp = 100, Kd = 10, Ki = 50; //These Values Come From Mike's MATLAB File
unsigned int T1_Prev, T2_Prev, T3_Prev, T4_Prev;
unsigned int T1_Current = 0, T2_Current = 0, T3_Current = 0, T4_Current = 0;
unsigned int T1, T2, T3, T4;
unsigned int u, u2;
unsigned int difference1, difference2, difference3, difference4;
unsigned int w1, w2, w3, w4;
unsigned int count_temp1, count_temp2, count_temp3, count_temp4;
unsigned int count1, count2, count3, count4;
unsigned int test = 0;

//PWM Service Routine
void TPM1_IRQHandler() {

	// PID
	speed_motor1 = PID_Control1();

	// Set Duty Cycle PWM
	TPM1_C0V = TPM_CnV_VAL(0x64);// + speed_motor2;  //The duty cycle is equal to 0xFF/(speed_motor2)*100%

	//Set Motor Direction
	GPIOB_PDOR |= (1 << 8); //Set as Logic 1 Output

	//Reset Interrupt Flag
	TPM1_SC |= TPM_SC_TOF_MASK; //Resetting The Timer Overflow Flag

}

//PWM Service Routine
void TPM2_IRQHandler() {

	// PID
	//speed_motor0 = PID_Control0();

	// Set PWM
	TPM2_C0V = TPM_CnV_VAL(0x32);//+ speed_motor0;  //The duty cycle is equal to 0xFF/(speed_motor0)*100%

	//Set Motor Direction
	GPIOB_PDOR |= (1 << 9); //Set as Logic 1 Output

	// Reset Interrupt Flag
	TPM2_SC |= TPM_SC_TOF_MASK; //Resetting The Timer Overflow Flag

}

void TPM0_IRQHandler() {

	if (TPM0_STATUS & TPM_STATUS_CH0F_MASK) {

		T3_Current = TPM0_C0V; //Current count value

		if(T3_Current > T3_Prev){
			T3 = T3_Current - T3_Prev;//Regular calculation
		}
		else if (T3_Prev > T3_Current){
			T3 = T3_Prev - T3_Current; //In case of Overflow
			count3 = count_temp3; //capture number of
			count_temp3 = 0; //Reset temporary counter
		}
		else if (T3_Prev == T3_Current){
			T3 = 0; //No Change
		}
		else{

		}

		T3_Prev = T3_Current; //Update Previous to Current
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		count_temp3++; //Increment counter
		TPM0_STATUS |= TPM_STATUS_CH0F_MASK; //Reset Channel 0 Event
	}

	if (TPM0_STATUS & TPM_STATUS_CH1F_MASK) {

		T4_Current = TPM0_C1V; //Current Time

		if(T4_Current > T4_Prev){
			T4 = T4_Current - T4_Prev; //Period of Square Wave
		}
		else if(T4_Prev > T4_Current){
			T4 = T4_Prev - T4_Current; //In case of Overflow
			count4 = count_temp4; //capture number of
			count_temp4 = 0; //Reset temporary counter
		}
		else if(T4_Prev == T4_Current){
			T4 = 0; //No Change
		}
		else{

		}

		T4_Prev = T4_Current; //Temporary Variable
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		count_temp4++; //Increment counter
		TPM0_STATUS |= TPM_STATUS_CH1F_MASK; //Reset Channel 1 Event
	}

	//encoder1_data1 = GPIOA_PDIR & (1 << 1); //Read Data
	//encoder1_data2 = GPIOA_PDIR & (1 << 3); //Read Data

	if (TPM0_STATUS & TPM_STATUS_CH2F_MASK) {

		T1_Current = TPM0_C2V; //Current Time

		if(T1_Current > T1_Prev){
			T1 = T1_Current - T1_Prev; //Period of Square Wave
		}
		else if(T1_Prev > T1_Current){
			T1 = T1_Prev - T1_Current; //In case of Overflow
			count1 = count_temp1; //capture number of
			count_temp1 = 0; //Reset temporary counter
		}
		else if(T1_Prev == T1_Current){
			T1 = 0; //No Change
		}
		else{

		}

		T1_Prev = T1_Current; //Temporary Variable
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		count_temp1++; //Increment counter
		TPM0_STATUS |= TPM_STATUS_CH2F_MASK; //Reset Channel 1 Event
	}

	if (TPM0_STATUS & TPM_STATUS_CH3F_MASK) {
	//15.625 kHz

		T2_Current = TPM0_C3V; //Current Time

		if(T2_Current > T2_Prev){
			T2 = T2_Current - T2_Prev; //Period of Square Wave
		}
		else if(T2_Prev > T2_Current){
			T2 = T2_Prev - T2_Current; //In case of Overflow
			count2 = count_temp2; //capture number of
			count_temp2 = 0; //Reset temporary counter
		}
		else if(T2_Prev == T2_Current){
			T2 = 0; //No Change
		}
		else{

		}

		T2_Prev = T2_Current; //Temporary Variable
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		count_temp2++; //Increment counter
		TPM0_STATUS |= TPM_STATUS_CH3F_MASK; //Reset Channel 1 Event
	}

}

void init_TPM() {
	//Init TPM2
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK; // Enable TPM0
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //Initialize PORT A
	PORTA_PCR1 |= PORT_PCR_MUX(3); //PTA1 Outputting TPM2_CH0
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC_MASK; // TPM Module Uses MCGIRCLK Clock

	TPM2_C0SC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK); // choose the channel 5 mode as Center Aligned PWM mode
	TPM2_MOD = 0xFF;
	//TPM0_MOD = (uint32_t)((TPM0_MOD & (uint32_t)~(uint32_t)(TPM_MOD_MOD(0xFFF6))) | (uint32_t)(TPM_MOD_MOD(0x09)));  // TPM MOD =9 ,it means the TPM cycle is 10us
	TPM2_SC = (uint32_t) ((TPM2_SC & (uint32_t) ~(uint32_t) (
	TPM_SC_DMA_MASK |
	TPM_SC_CPWMS_MASK | TPM_SC_CMOD(0x02))) | (uint32_t) (
	TPM_SC_TOF_MASK |
	TPM_SC_TOIE_MASK | TPM_SC_CMOD(0x01)));
	NVIC_EnableIRQ(TPM2_IRQn); // Enable TPM0 Interrupt
	NVIC_SetPriority(TPM2_IRQn, 2); //Priority level 2

	//Init TPM1
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK; // Enable TPM1
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // Initialize PORT A
	PORTA_PCR12 |= PORT_PCR_MUX(3); //PTE20 Outputting TPM1_CH0

	TPM1_C0SC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK); // choose the channel 0 mode as Center Aligned PWM mode
	//TPM1_MOD = (uint32_t)((TPM1_MOD & (uint32_t)~(uint32_t)(TPM_MOD_MOD(0xFFF6))) | (uint32_t)(TPM_MOD_MOD(0x09)));  // TPM MOD =9 ,it means the TPM cycle is 10us
	//TPM1_MOD = 0x9;
	TPM1_MOD = 0xFF;
	TPM1_SC = (uint32_t) ((TPM1_SC & (uint32_t) ~(uint32_t) (
	TPM_SC_DMA_MASK |
	TPM_SC_CPWMS_MASK | TPM_SC_CMOD(0x02))) | (uint32_t) (
	TPM_SC_TOF_MASK |
	TPM_SC_TOIE_MASK | TPM_SC_CMOD(0x01)));
	NVIC_EnableIRQ(TPM1_IRQn); //Enable TPM1 Interrupt
	NVIC_SetPriority(TPM1_IRQn, 2); //Priority level 2

	//Init TPM0
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK; // Enable TPM0
	TPM0_MOD |= 0xFFFF; //Overflow value
	//TPM0_SC |= TPM_SC_PS(3); //Change this value around to try and get mes_vel reasonable, with 3 clock should be 1953.125 Hz
	TPM0_SC |= TPM_SC_TOF_MASK;
	TPM0_SC |= TPM_SC_DMA_MASK;

	//Init TPM0_CH0
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK; // Enable TPM0
	TPM0_C0SC |= TPM_CnSC_ELSA_MASK; // choose the channel 0 mode as rising edge
	TPM0_C0SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C0SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access

	//Init TPM0_CH1
	TPM0_C1SC |= TPM_CnSC_ELSA_MASK; // choose the channel 0 mode as rising edge
	TPM0_C1SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C1SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access

	//Init TPM0_CH2
	TPM0_C2SC |= TPM_CnSC_ELSA_MASK; // choose the channel 0 mode as rising edge
	TPM0_C2SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C2SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access

	//Init TPM0_CH3
	TPM0_C3SC |= TPM_CnSC_ELSA_MASK; // choose the channel 0 mode as rising edge
	TPM0_C3SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C3SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access


	TPM0_SC |= TPM_SC_CMOD(0x01); //Enable Timer
	NVIC_EnableIRQ(TPM0_IRQn); // Enable TPM0 Interrupt
	NVIC_SetPriority(TPM0_IRQn, 1); //Priority level 1

}

void init_MCG() {
	MCG_C1 = MCG_C1_IRCLKEN_MASK; //Allows Use as IRCLKEN Clock
	MCG_C2 = MCG_C2_IRCS_MASK; //Fast Internal Clock is Selected
	MCG_SC = MCG_SC_FCRDIV_MASK & 0x01; //Divide Clock by a Factor of 2 = 15.625 KHz
}

void init_Encoder() {

	//First Encoder on PTC3 and PTC4
	//Encoder 1
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //Initialize Port C
	PORTC_PCR3 |= PORT_PCR_MUX(4); //MUX For TPM0_CH2
	PORTC_PCR4 |= PORT_PCR_MUX(4); //MUX For TPM0_CH3
	PORTC_PCR3 |= 0x90000; //Interrupt on Rising Edge & Configured as GPIO PTD3
	PORTC_PCR4 |= 0x90000; //Interrupt on Rising Edge & Configured as GPIO PTD1


	//Encoder 2 on PTC1 & PTC2
	PORTC_PCR1 |= PORT_PCR_MUX(4); //MUX for TPM0_CH0
	PORTC_PCR2 |= PORT_PCR_MUX(4); //MUX For TPM0_CH1
	PORTC_PCR1 |= 0x90000; //Interrupt on Rising Edge
	PORTC_PCR2 |= 0x90000; //Interrupt on Rising Edge & Configured as GPIO PTA2

}

// This Function Initializes Two GPIO Ports (PTB8 & PTB9) To Be Used As The Direction Control For The Two Motors
void init_direction() {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Initialize Port B
	PORTB_PCR8 |= PORT_PCR_MUX(1); //GPIO
	PORTB_PCR9 |= PORT_PCR_MUX(1); //GPIO
	GPIOB_PDDR |= (1 << 8); //Set as Output
	GPIOB_PDDR |= (1 << 9); //Set as Output
}

// This Function is For Testing Purposes
void init_led() {

	//Red LED
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Initialize Port B
	PORTB_PCR18 = 0x100; //Red LED
	GPIOB_PDDR |= (1 << 18); //Setting as Output

	//Green LED
	PORTB_PCR19 = 0x100; //Green LED
	GPIOB_PDDR |= (1 << 19); //Setting as Output
}

//In order for this function to work the mes_vel calculation needs to use the input ports Tx value,
//and the Tx for the I_Term & D_Term must be different from the first, possibly needs no input for it to work
int PID_Control0() {

	des_vel = 0x30;
	mes_vel = (count2 * 2 * 3 * 15625) / (1398 * T2); //Attempting to get around floating point error T*1us

	e = des_vel - mes_vel; //Error
	I_Term = I_Term + e * T2; //Previous error + error*time step, doesnt work with period for some reason
	D_Term = (e - e_Prev) / T2; //Slope, doesnt work with the period for some reason
	e_Prev = e; //Previous Error
	if (I_Term < I_Term_Min)
		I_Term = I_Term_Min;
	if (I_Term > I_Term_Max)
		I_Term = I_Term_Max;

	u = e*1 + I_Term*1 + D_Term*1;
	//u = u/256;//u*0.00008/256;

	//Limit u if it is too large
	if (u > 0xFF) u = 0xFF;//Limit Output if too large

	//if(u >= 0) Dir = 1; else Dir = 0;

	return u;
}

int PID_Control1() {

	des_vel2 = 0x60;
	mes_vel2 = (count3 * 2 * 3 * 15625) / (1398 * T3); //Attempting to get around floating point error T*1us

	e2 = des_vel2 - mes_vel2; //Error
	I_Term2 = I_Term + e2 * T3; //Previous error + error*time step, doesnt work with period for some reason
	D_Term2 = (e2 - e_Prev2) / T3; //Slope, doesnt work with the period for some reason
	e_Prev2 = e2; //Previous Error
	if (I_Term2 < I_Term_Min)
		I_Term2 = I_Term_Min;
	if (I_Term2 > I_Term_Max)
		I_Term2 = I_Term_Max;

	u2 = e2*1 + I_Term2*1 + D_Term2*1;
	//u = u*0.00008/256;

	//Limit u if it is too large
	if (u2 > 0xFF) u2 = 0xFF;//Limit Output if too large

	//if(u >= 0) Dir = 1; else Dir = 0;

	return u2;
}

void init_delay() {
	//Init TPM2
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK; // Enable TPM2

	//TPM2_C0SC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK); // choose the channel 0 mode as Center Aligned PWM mode
	TPM2_MOD = 0x9;
	TPM2_SC = (uint32_t) ((TPM2_SC & (uint32_t) ~(uint32_t) (
	TPM_SC_DMA_MASK |
	TPM_SC_CPWMS_MASK | TPM_SC_CMOD(0x02) | TPM_SC_PS(0x04))) | (uint32_t) (
	TPM_SC_TOF_MASK | TPM_SC_CMOD(0x01) | TPM_SC_PS(0x03)));

}
//This function delays by input*1us
int delay() {

}


void UART_Interface_Init(int Baud){
	uint16_t tmp;
	//enable uart clock
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	//2 multiplex pin
	PORTA_PCR1 |= PORT_PCR_MUX(2);//rx
	PORTA_PCR2 |= PORT_PCR_MUX(2);//tx

	    UART0_C1 = 0;
	    UART0_C2 &=~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
	   // UART0_C2 = UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK;
	    UART0_C3 = 0;
	    UART0_C4 = 0;
	    UART0_C5 = 0;
	    UART0_S2 = 0;
	    SIM_SOPT2 &= ~(SIM_SOPT2_PLLFLLSEL_MASK);	/* MCGFLLCLK			*/
	    SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);		/* Using MCGFLLPLL		*/
	    tmp = SystemCoreClock / Baud;
	    UART0_BDH=0;
	    	UART0_BDL=0X88;
	    	UART0_C2 |= UARTLP_C2_RE_MASK;
	    		UART0_C2 |= UARTLP_C2_TE_MASK;
}
// PUT_CHAR                     GET_CHAR                  PTUSTR



char UART0_Getchar(){
		//wait until character has been received
		while (!(UART0_S1 & UARTLP_S1_RDRF_MASK));
		// return the 8-bit data from the receiver
		return UART0_D;
}
void UART0_Putchar(char ch){
		//wait until space is available in the FIFO
		while(!(UART0_S1 & UARTLP_S1_TDRE_MASK));
		//send the character
		UART0_D=ch;

}


int main(void) {

	init_MCG();
	init_TPM();
	init_Encoder();
	init_direction();
	//init_led();

	//UART_Interface_Init(9600);

	T1_Prev = TPM0_CNT; //Initial TPM0 Value
	T2_Prev = TPM0_CNT; //Initial TPM0 Value
	T3_Prev = TPM0_CNT; //Initial TPM0 Value
	T4_Prev = TPM0_CNT; //Initial TPM0 Value


	for (;;) {
		//UART0_Putchar(0x30);
		//UART0_Putchar(',');
		//UART0_Putchar((char)u);
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
