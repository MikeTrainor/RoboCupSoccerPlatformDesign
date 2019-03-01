
#include "MKL25Z4.h"
#include "math.h"

// Pin Assignments Page 161/807
// PWM Module Page 547/807
// GPIO Registers Page 776/807
// http://makeatronics.blogvelot.com/2013/02/efficiently-reading-quadrature-with.html
// https://www.dialog-semiconductor.com/sites/default/files/an-cm-250_position_and_speed_control_of_a_dc_motor_using_analog_pid_controller.pdf

/*PORTS IN USE
 * PTC1, PTC2 Encoder 1
 * PTC3, PTC4 Encoder 2
 * PTA1, PTA12 = PWM Output
 * PTB8, PTB9 Direction 1, Direction 2
 */

/*General Information
 * NO LOAD INFORMATION & Step
 * Minimum speed obtainable for the motor is approximately 43 RPM
 * Maximum speed obtainable for the motor is approximately 410 RPM although the error gets rough
 *
 * NO LOAD INFORMATION & Ramp
 * Minimum speed obtainable for the motor is approximately 43 RPM
 * Maximum speed obtainable for the motor is approximately 410 RPM although the error gets rough
 *
 * UNDER LOAD INFORMATION
 *
 */

// Global Variables
float des_vel, mes_vel = 0, des_vel_prev, des_vel2, mes_vel2 = 0, des_vel_prev2;
float e = 0, e_Prev = 0, e2 = 0, e_Prev2 = 0;
float P_Term = 0, I_Term = 0, I_Term2 = 0, I_Term_prev, I_Term_prev2, D_Term = 0, P_Term2 = 0, D_Term2 = 0;
int I_Term_Min = -0xFFFF, I_Term_Max = 0xFFFF;
float speed_motor0 = 0, speed_motor1 = 0;
float Kp = 5, Kd = 5, Ki = 1.05; //Current Best is Kp = 5, Kd = 5, Ki = 1.05, Prev Best is Kp = 5, Kd = 5, Ki = 1.1, this is for step input
//float Kp = 50, Kd = 10, Ki = 15;
unsigned int T1_Prev, T2_Prev, T3_Prev, T4_Prev;
unsigned int T1_Current = 0, T2_Current = 0, T3_Current = 0, T4_Current = 0;
unsigned int T1, T2, T3, T4;
float u = 0, u2 = 0;
float r = 50, r2 = 50;
unsigned int direction = 0, direction2 = 0; //Values for direction
float ramp_start = 50, ramp_end = 410, ramp_rate = 8, ramp_rate2 = 8; //Ramp rate for motors could be as low as 0.2 or as high as 410?
int quad_count1 = 0, quad_count2 = 0;
float dt1 = 0, dt2 = 0; //dt is an ideal value calculated from 255/15625
int test_count = 0;
int flag1 = 1, flag2 = 1; //Interrupt flags
char cout;
char hexaDeciNum[4] = {0,0,0,0};
int time = 0;
int Beta = 2, FP_Shift = 1;
signed long SmoothDataFP, SmoothDataINT;
int PID_count1 = 0, PID_count2 = 0;
//Previous Best is 200Hz
//Current Best is 500Hz, which can get up to 250RPM

//PWM Service Routine
void TPM1_IRQHandler() {

	PID_count1 += 1;
	if(flag1 == 1){
		// PID
		speed_motor0 = PID_Control0();

		// Set Duty Cycle PWM
		TPM1_C0V = TPM_CnV_VAL(0x00) + speed_motor0;  //The duty cycle is equal to 0xFF/(speed_motor0)*100%

		//Set Motor Direction
		GPIOB_PDOR |= (1 << 8); //Set as Logic 1 Output
	}

	//Reset Interrupt Flag
	TPM1_SC |= TPM_SC_TOF_MASK; //Resetting The Timer Overflow Flag

}

//PWM Service Routine
void TPM2_IRQHandler() {

	PID_count2 += 1; //Increment PID_count2
	if(flag2 == 1){
		// PID
		speed_motor1 = PID_Control1();

		// Set PWM
		TPM2_C0V = TPM_CnV_VAL(0x00) + speed_motor1;  //The duty cycle is equal to 0xFF/(speed_motor1)*100%

		//Set Motor Direction
		GPIOB_PDOR |= (1 << 9); //Set as Logic 1 Output
	}


	// Reset Interrupt Flag
	TPM2_SC |= TPM_SC_TOF_MASK; //Resetting The Timer Overflow Flag

}

void TPM0_IRQHandler() {

	//////////////////////////////////////////////////////////////////
	//Encoder 1

	if (TPM0_STATUS & TPM_STATUS_CH0F_MASK) {

		T3_Current = TPM0_C0V; //Current count value

		if(T3_Current > T3_Prev){
			T3 = T3_Current - T3_Prev;//Regular calculation
		}
		else if (T3_Prev > T3_Current){
			T3 = TPM0_MOD - T3_Prev + T3_Current; //In case of Overflow
			TPM0_SC |= TPM_SC_TOF_MASK;
		}
		else if (T3_Prev == T3_Current){
			T3 = 0; //No Change
		}

		//Check other state, increment if CH1 leads CH0
		if(TPM0_STATUS & TPM_STATUS_CH1F_MASK){
			quad_count1++;
		}

		T3 = LPF(T3);//Filter the Data
		T3_Prev = T3_Current; //Update Previous to Current
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		TPM0_STATUS |= TPM_STATUS_CH0F_MASK; //Reset Channel 0 Event
		flag2 = 1; //Set flag to update PID
	}

	if (TPM0_STATUS & TPM_STATUS_CH1F_MASK) {

		T4_Current = TPM0_C1V; //Current Time

		if(T4_Current > T4_Prev){
			T4 = T4_Current - T4_Prev; //Period of Square Wave
		}
		else if(T4_Prev > T4_Current){
			T4 = TPM0_MOD - T4_Prev + T4_Current; //In case of Overflow
			TPM0_SC |= TPM_SC_TOF_MASK; //Reset overflow timer
		}
		else if(T4_Prev == T4_Current){
			T4 = 0; //No Change
		}

		//Check other state, decrement if CH0 leads CH1
		if(TPM0_STATUS & TPM_STATUS_CH0F_MASK){
			quad_count1--;
		}

		T4_Prev = T4_Current; //Temporary Variable
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		TPM0_STATUS |= TPM_STATUS_CH1F_MASK; //Reset Channel 1 Event
	}

	if(quad_count1 < 0){
		direction = -1;
	}
	else if(quad_count1 > 0){
		direction = 1;
	}
	else{
		direction = 0;
	}

	//////////////////////////////////////////////////////////////////
	//Encoder 2

	if (TPM0_STATUS & TPM_STATUS_CH2F_MASK) {

		T1_Current = TPM0_C2V; //Current Time

		if(T1_Current > T1_Prev){
			T1 = T1_Current - T1_Prev; //Period of Square Wave
		}
		else if(T1_Prev > T1_Current){
			T1 = TPM0_MOD - T1_Prev + T1_Current; //In case of Overflow
			TPM0_SC |= TPM_SC_TOF_MASK; //Reset overflow timer
		}
		else if(T1_Prev == T1_Current){
			T1 = 0; //No Change
		}

		//Check other state, increment if CH3 leads CH2
		if(TPM0_STATUS & TPM_STATUS_CH3F_MASK){
			quad_count2++;
		}

		T1_Prev = T1_Current; //Temporary Variable
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		TPM0_STATUS |= TPM_STATUS_CH2F_MASK; //Reset Channel 1 Event
	}

	if (TPM0_STATUS & TPM_STATUS_CH3F_MASK) {

		T2_Current = TPM0_C3V; //Current Time

		if(T2_Current > T2_Prev){
			T2 = T2_Current - T2_Prev; //Period of Square Wave
		}
		else if(T2_Prev > T2_Current){
			T2 = TPM0_MOD - T2_Prev + T2_Current; //In case of Overflow
			TPM0_SC |= TPM_SC_TOF_MASK; //Reset overflow timer
		}
		else if(T2_Prev == T2_Current){
			T2 = 0; //No Change
		}

		//Check other state, increment if CH2 leads CH3
		if(TPM0_STATUS & TPM_STATUS_CH2F_MASK){
			quad_count2--;
		}

		T2 = LPF(T2);//Filter the Data
		T2_Prev = T2_Current; //Temporary Variable
		TPM0_SC |= TPM_SC_TOF_MASK; //Reset Flag
		TPM0_STATUS |= TPM_STATUS_CH3F_MASK; //Reset Channel 1 Event
		flag1 = 1;//Set Flag to update PID
	}

	if(quad_count2 < 0){
		direction2 = -1;
	}
	else if(quad_count2 > 0){
		direction2 = 1;
	}
	else{
		direction2 = 0;
	}


}

void init_TPM() {

	NVIC_DisableIRQ(TPM2_IRQn); // Enable TPM0 Interrupt
	NVIC_DisableIRQ(TPM1_IRQn); // Enable TPM0 Interrupt
	NVIC_DisableIRQ(TPM0_IRQn); // Enable TPM0 Interrupt
	NVIC_SetPriority(TPM2_IRQn, 2); //Priority level 2
	NVIC_SetPriority(TPM1_IRQn, 2); //Priority level 2
	NVIC_SetPriority(TPM0_IRQn, 1); //Priority level 2

	//Init TPM2
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK; // Enable TPM0
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //Initialize PORT A
	PORTA_PCR1 |= PORT_PCR_MUX(3); //PTA1 Outputting TPM2_CH0
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC_MASK; // TPM Module Uses MCGIRCLK Clock

	TPM2_C0SC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK); // choose the channel 5 mode as Center Aligned PWM mode
	TPM2_MOD = 0x1F40; //500 Hz PWM
	TPM2_SC = (uint32_t) ((TPM2_SC & (uint32_t) ~(uint32_t) (
	TPM_SC_DMA_MASK |
	TPM_SC_CPWMS_MASK | TPM_SC_CMOD(0x02))) | (uint32_t) (
	TPM_SC_TOF_MASK |
	TPM_SC_TOIE_MASK | TPM_SC_CMOD(0x01)));
	TPM2_SC |= TPM_SC_PS_MASK & 0x7; //Divide clock down by 127

	//Init TPM1, Modified to PTE20, PTE21
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK; // Enable TPM1
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; // Initialize PORT C
	PORTA_PCR12|= PORT_PCR_MUX(3); //PTA12 Outputting TPM1_CH0

	TPM1_C0SC = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK); // choose the channel 0 mode as Center Aligned PWM mode
	TPM1_MOD = 0x1F40; //500 Hz PWM
	//TPM1_SC |= TPM_SC_PS_MASK & 0x1; //Divide clock down by 127
	TPM1_SC = (uint32_t) ((TPM1_SC & (uint32_t) ~(uint32_t) (
	TPM_SC_DMA_MASK |
	TPM_SC_CPWMS_MASK | TPM_SC_CMOD(0x02))) | (uint32_t) (
	TPM_SC_TOF_MASK |
	TPM_SC_TOIE_MASK | TPM_SC_CMOD(0x01)));
	TPM1_SC |= TPM_SC_PS_MASK & 0x7; //Divide clock down by 127


	//Init TPM0
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK; // Enable TPM0
	TPM0_SC |= TPM_SC_PS_MASK & 0x2; //4MHz/2 = 2MHz
	TPM0_SC |= TPM_SC_TOF_MASK;
	TPM0_SC |= TPM_SC_DMA_MASK;

	//Init TPM0_CH0
	TPM0_C0SC |= (TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK); // choose the channel 0 mode as rising & Falling edges
	TPM0_C0SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C0SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access

	//Init TPM0_CH1
	TPM0_C1SC |= (TPM_CnSC_ELSA_MASK); // choose the channel 0 mode as rising
	TPM0_C1SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C1SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access

	//Init TPM0_CH2
	TPM0_C2SC |= (TPM_CnSC_ELSA_MASK); // choose the channel 0 mode as rising
	TPM0_C2SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C2SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access

	//Init TPM0_CH3
	TPM0_C3SC |= (TPM_CnSC_ELSA_MASK); // choose the channel 0 mode as rising
	TPM0_C3SC |= TPM_CnSC_CHIE_MASK; //Channel interrupt enable
	TPM0_C3SC |= TPM_CnSC_DMA_MASK; //Allow Direct Memory Access


	TPM0_SC |= TPM_SC_CMOD(0x01); //Enable Timer
	TPM0_MOD = 0xFFFF; //Overflow value

	NVIC_EnableIRQ(TPM2_IRQn); // Enable TPM0 Interrupt
	NVIC_EnableIRQ(TPM1_IRQn); //Enable TPM1 Interrupt
	NVIC_EnableIRQ(TPM0_IRQn); //Enable TPM0 Interrupt

}

//Mess around with the clock to make PWM & Encoder input work at the same time
void init_MCG() {
	MCG_C1 = MCG_C1_IRCLKEN_MASK; //Allows Use as IRCLKEN Clock
	MCG_C2 = MCG_C2_IRCS_MASK; //Fast Internal Clock is Selected
	MCG_SC = MCG_SC_FCRDIV_MASK & 0x01; //Divide Clock by a Factor of 1 = 4 MHz
	//MCG_SC = MCG_SC_FCRDIV_MASK; //Divide Clock by a Factor of 7 = 4 MHz / 128
}

void init_Encoder() {

	//First Encoder on PTC3 and PTC4
	//Encoder 1
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //Initialize Port C
	PORTC_PCR3 |= PORT_PCR_MUX(4); //MUX For TPM0_CH2
	PORTC_PCR4 |= PORT_PCR_MUX(4); //MUX For TPM0_CH3
	PORTC_PCR3 |= 0x90000; //Interrupt on Rising Edge
	PORTC_PCR4 |= 0x90000; //Interrupt on Rising Edge


	//Encoder 2 on PTC1 & PTC2
	PORTC_PCR1 |= PORT_PCR_MUX(4); //MUX for TPM0_CH0
	PORTC_PCR2 |= PORT_PCR_MUX(4); //MUX For TPM0_CH1
	PORTC_PCR1 |= 0x90000; //Interrupt on Rising Edge
	PORTC_PCR2 |= 0x90000; //Interrupt on Rising Edge

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

	dt1 = PID_count1*0.51;//Count*one Periodic timer, 255/500 = 0.51. 500 is the PWM Frequency and 255 is the register size
	//des_vel = 50;//Set the desired value
	mes_vel = (2 * 3 * 2e6 * (30/3)) / (700 * T2 * 2); //Calculate the measured speed

	if(T2 == 0){
		mes_vel = 0; //Accounting for the case of having desired velocity equal to zero
	}


	if(r > ramp_end){
		r = ramp_end;
	}

	//Ramping
	if(des_vel <= 50){
		ramp_rate = 8; //Default ramp rate
	}
	else{
		ramp_rate = des_vel/75; //Faster Ramp Rate
	}

	if(r < des_vel){
		r = r + ramp_rate; //Ramp up

		if(r > des_vel){
			r = des_vel; //Limit if it over shoots the desired
		}
	}
	if(r > des_vel){
		r = r - ramp_rate; //Ramp down

		if(r < des_vel){
			r = des_vel; //Limit if it under shoots the desired
		}
	}


	e = des_vel - mes_vel; //Error
	D_Term = (e - e_Prev) / (dt1); //Slope


	u = fabs(e*Kp + I_Term*Ki + D_Term*Kd);

	if(fabs(u) >= 0x1F40){
		u = 0x1F40;//Max Duty Cycle
	}
	else{
		I_Term = I_Term + e*dt1; //Previous error + error*time step, scaling factor 10e2
	}

	flag1 = 0; //Reset Flag
	PID_count1 = 0; //Reset PID_count
	e_Prev = e; //Previous Error

	return u;
}

int PID_Control1() {

	dt2 = PID_count2*0.51;//Count*one Periodic timer, 255/500 = 0.51. 500 is the PWM Frequency and 255 is the register size
	//des_vel2 = 45;
	if(time >= 600){
		//des_vel2 = 0;
	}

	mes_vel2 = (2 * 3 * 2e6 * (30/3)) / (700 * T3 * 2); // Motor Speed in RPM at the wheel

	if(T3 == 0){
		mes_vel2 = 0; //Accounting for the case of having desired velocity equal to zero
	}

	//Ramping
	if(des_vel2 <= 50){
		ramp_rate2 = 8; //Default ramp rate
	}
	else{
		ramp_rate2 = des_vel2/75; //Faster Ramp Rate
	}

	if(r2 > ramp_end){
		r2 = ramp_end;
	}

	if(r2 < des_vel2){
		r2 = r2 + ramp_rate2; //Ramp up

		if(r2 > des_vel2){
			r2 = des_vel2; //Limit if it over shoots the desired
		}
	}
	if(r2 > des_vel2){
		r2 = r2 - ramp_rate2; //Ramp down

		if(r2 < des_vel2){
			r2 = des_vel2; //Limit if it under shoots the desired
		}
	}

	e2 = r2 - mes_vel2; //Error, Ensure Negative Feedback
	D_Term2 = (e2 - e_Prev2)/dt2; //Slope

	u2 = fabs((e2*Kp + I_Term2*Ki + D_Term2*Kd));

	if(fabs(u2) >= 0x1F40){
		u2 = 0x1F40;//Max Duty Cycle
	}
	else{
		I_Term2 = I_Term2 + e2*dt2; //Previous error + error*time step, scaling factor 10e2
	}

	e_Prev2 = e2; //Previous Error
	flag2 = 0;//Reset flag
	PID_count2 = 0;//Reset PID_count2

	return u2;
}

//Input Delay length in ms
void init_delay() {

    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Clock Enabled
    LPTMR0_CSR = 0;                     // Reset LPTMR settings
   // LPTMR0_CMR = length_ms;             // Set compare value (in ms)

    // Use 1kHz LPO with no prescaler
    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;

}

//Input Delay length in ms
int delay(unsigned int length_ms) {

    // Start the timer and wait for it to reach the compare value
    LPTMR0_CSR = LPTMR_CSR_TEN_MASK;
	LPTMR0_CMR = length_ms;             // Set compare value (in ms)

    // Start the timer and wait for it to reach the compare value
   // LPTMR0_CSR = LPTMR_CSR_TEN_MASK;

    while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)){}

    LPTMR0_CSR = 0; //Turn off timer

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

// function to convert decimal to hexadecimal
void decToHexa(int n)
{

    // counter for hexadecimal number array
    int i = 0;
    int j = 0;
    while(n!=0)
    {
        // temporary variable to store remainder
        int temp  = 0;

        // storing remainder in temp variable.
        temp = n % 16;

        // check if temp < 10
        if(temp < 10)
        {
            hexaDeciNum[i] = temp + 48;
            i++;
        }
        else
        {
            hexaDeciNum[i] = temp + 55;
            i++;
        }

        n = n/16;
    }

    // printing hexadecimal number array in reverse order
    for(j=i-1; j>=0; j--){
        cout << hexaDeciNum[j];
    }


}

int LPF(int val){

	val <<= FP_Shift; // Shift to fixed point
	SmoothDataFP = (SmoothDataFP<< Beta)-SmoothDataFP;
	SmoothDataFP += val;
	SmoothDataFP >>= Beta;
	// Don't do the following shift if you want to do further
	// calculations in fixed-point using SmoothData
	SmoothDataINT = SmoothDataFP>> FP_Shift;
	return SmoothDataINT;
}

int main(void) {

	init_MCG();
	init_Encoder();
	init_TPM();
	init_direction();
	//init_led();
	init_delay();

	UART_Interface_Init(9600);

	T1_Prev = TPM0_CNT; //Initial TPM0 Value
	T2_Prev = TPM0_CNT; //Initial TPM0 Value
	T3_Prev = TPM0_CNT; //Initial TPM0 Value
	T4_Prev = TPM0_CNT; //Initial TPM0 Value


	for (;;) {

		//decToHexa(abs((int)mes_vel2));
		//UART0_Putchar(hexaDeciNum[3]); //Needed is the velocity is above  RPM
		//UART0_Putchar(hexaDeciNum[2]); //Needed is the velocity is above 255 RPM
		//UART0_Putchar(hexaDeciNum[1]);
		//UART0_Putchar(hexaDeciNum[0]);
		//time = time + 1;
		des_vel = UART0_Getchar();//Right Motor Speed
		des_vel2 = UART0_Getchar();//Left Motor Speed

	}

	return 0;
}
