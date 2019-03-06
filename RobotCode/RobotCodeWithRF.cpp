// Working Robot Code
// Version 1.1
// Authors: Mike Trainor, Koceila Cherfouh
// Date: March 5, 2018

#include "mbed.h"
#include "nRF24L01P.h"
#include "rtos.h"
#include "Serial.h"
#include "QEI.h"

//Function Prototypes
void Encoder_ISR(void); //Handler for Encoder
void EncoderThread(void const *argument); //Handler for Encoder
void PID1(void const *argument); //PID Control for motor 1
void PID2(void const *argument); //PID Control for motor 2
void PeriodicInterruptISR(void); //Periodic Timer Interrupt Serive Routine for Motor 1
void PeriodicInterrupt2ISR(void); //Periodic Timer Interrupt Serive Routine for Motor 2
void AssignData();

//Processes and Threads
osThreadId EncoderID, Encoder2ID, PiControlId, PiControl2Id; // Thread ID's
osThreadDef(PID1, osPriorityRealtime, 1024); // Declare PiControlThread as a thread, highest priority
osThreadDef(PID2, osPriorityRealtime, 1024); // Declare PiControlThread2 as a thread, highest priority
osThreadDef(EncoderThread, osPriorityRealtime, 1024); // Declare PiControlThread as a thread, highest priority

Serial pc(USBTX, USBRX); // tx, rx
nRF24L01P my_nrf24l01p(PTD2, PTD3, PTD1, PTD0, PTE0, PTD5);    // mosi, miso, sck, csn, ce, irq
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);

// Data Packets Requirements
#define StartBit        0xFF
#define IDbit           0x01
#define EndBit          0xFF

#define TRANSFER_SIZE   8

char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE];
int txDataCnt = 0;
int rxDataCnt = 0;


//Port Configuration
//Motor Directions
DigitalOut dir_motor1(PTE4); //Direction Motor 1
DigitalOut dir_motor2(PTE5); //Direction Motor 2

//PWM Outputs
PwmOut PWM_motor1(PTE20); //PWM Motor2
PwmOut PWM_motor2(PTE22); //PWM Motor1

//Kick Output
DigitalOut kick(PTB10); //Kick command
//Encoder
QEI motor1(PTD4, PTA12, NC, 16, QEI::X4_ENCODING); //QEI for motor 1
QEI motor2(PTA4, PTA5, NC, 16, QEI::X4_ENCODING); //QEI for motor 2
//Timer Interrupts
Ticker PeriodicInt; // Declare a timer interrupt: PeriodicInt
Ticker PeriodicInt2; // Declare a timer interrupt: PeriodicInt2
Ticker Encoder_Tick; //Timer Interrupt for counting Pulses




//Declare Global Variables
float des_vel = 0, mes_vel = 0, des_vel2 = 0, mes_vel2 = 0;
float e = 0, e_Prev = 0, e2 = 0, e_Prev2 = 0;
float I_Term = 0, I_Term2 = 0, D_Term = 0, D_Term2 = 0;
float Kp = 5, Kd = 5, Ki = 1.05; //Current Best is Kp = 5, Kd = 5, Ki = 1.05, Prev Best is Kp = 5, Kd = 5, Ki = 1.1, this is for step input
float u = 0, u2 = 0;
float r = 50, r2 = 50;
float ramp_start = 50, ramp_end = 260, ramp_rate = 8, ramp_rate2 = 8; //Ramp rate for motors could be as low as 0.2 or as high as 410?
float dt = 0.05; //dt best = 0.05?
float pulses_motor1, pulses_motor2;
float PWM_Period = 2000; //in micro-seconds


//Testing Digital Input for LED
//DigitalOut led3(LED3);


int main() {

// The nRF24L01+ supports transfers from 1 to 32 bytes, but Sparkfun's
//  "Nordic Serial Interface Board" (http://www.sparkfun.com/products/9019)
//  only handles 4 byte transfers in the ATMega code.
    pc.baud(9600);  // Set max uart baud rate
    pc.printf("We have good serial communication! :D\n");

    //Initializing Threads for interrupts
    PiControlId = osThreadCreate(osThread(PID1), NULL); //Create Thread for PID1
    PiControl2Id = osThreadCreate(osThread(PID2), NULL); //Create Thread for PID2
    EncoderID = osThreadCreate(osThread(EncoderThread), NULL); //Create Encoder Thread
    
    //PWM Period
    PWM_motor1.period_us(PWM_Period); // This sets the PWM period to 2000 us = 500Hz
    PWM_motor2.period_us(PWM_Period); // This sets the PWM period to 2000 us = 500Hz

    //Periodic Interrupts
    PeriodicInt.attach(&PeriodicInterruptISR, dt); //Periodic timer interrupt every 0.02 seconds
    PeriodicInt2.attach(&PeriodicInterrupt2ISR, dt); //Periodic timer interrupt every 0.02 seconds
    Encoder_Tick.attach(&Encoder_ISR, 4*dt);

    my_nrf24l01p.powerUp();
    my_nrf24l01p.setAirDataRate(NRF24L01P_DATARATE_2_MBPS);

    // Display the (default) setup of the nRF24L01+ chip
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );

    pc.printf( "Type keys to test transfers:\r\n  (transfers are grouped into %d characters)\r\n", TRANSFER_SIZE );

    my_nrf24l01p.setTransferSize( TRANSFER_SIZE );

    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();
    
    
    
    
    char test = 0;
    //Loop Forever
    while (1) {
        /*
        // If we've received anything over the host serial link...
        if ( pc.readable() ) {

            // ...add it to the transmit buffer
            txData[txDataCnt++] = pc.getc();

            // If the transmit buffer is full
            if ( txDataCnt >= sizeof( txData ) ) {

                // Send the transmitbuffer via the nRF24L01+
                my_nrf24l01p.write( NRF24L01P_PIPE_P0, txData, txDataCnt );

                txDataCnt = 0;
            }

            // Toggle LED1 (to help debug Host -> nRF24L01+ communication)
            myled1 = !myled1;
        }*/

        // If we've received anything in the nRF24L01+...
        if ( my_nrf24l01p.readable() ) {

            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( NRF24L01P_PIPE_P0, rxData, sizeof( rxData ) );


            AssignData();
            // Display the receive buffer contents via the host serial link
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                pc.putc( rxData[i] );
            }

            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
            myled2 = !myled2;
        }
    }
}




void AssignData(){
    if((rxData[0] == StartBit) &&  (rxData[1] == IDbit) &&  (rxData[7] == EndBit)){
        // Transmitted Data
        des_vel = rxData[2];
        dir_motor1=rxData[3];
        des_vel2 = rxData[4];
        dir_motor2=rxData[5];
        kick    = rxData[6];
       // led3 = !led3;  
    }
}



// ******** Encoder Thread ********
void EncoderThread(void const *argument){
    while(true){
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal is received
        
        pulses_motor1 = abs(motor1.getPulses()); //Get number of pulses from motor1
        pulses_motor2 = abs(motor2.getPulses()); //Get number of pulses from motor2
    }
}

// ******** PID1 Thread ********
void PID1(void const *argument){
    
    while(true){
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal is received.
        //dir_motor1 = 1;
        //pulses_motor1 = abs(motor1.getPulses()); //Get number of pulses from motor1
        //des_vel = 30;
        mes_vel = (60*pulses_motor1) / (700 * dt * 4); //Calculate the measured speed
    
        if(pulses_motor1 == 0){
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
    
        if(mes_vel < 0){
            mes_vel = -mes_vel; //Ensure Positive Feedback
        }
        e = r - mes_vel; //Error
        D_Term = (e - e_Prev) / (dt); //Slope
        u = abs(e*Kp + I_Term*Ki + D_Term*Kd);
    
        if(abs(u) >= PWM_Period){
            u = PWM_Period;//Max Duty Cycle
        }
        else{
            if(des_vel != 0 && mes_vel != 0){
                I_Term = I_Term + e*dt; //Previous error + error*time step, scaling factor 10e2
            }
        }
    
        PWM_motor1.pulsewidth_us(abs(u)); //Set the Pulsewidth output
        e_Prev = e; //Previous Error
        motor1.reset(); //Reset the motor1 encoder    
    }
    
}

// ******** PID2 Thread ********
void PID2(void const *argument){
    
    while(true){
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal is received.
        
        //dir_motor2 = 1;
        //pulses_motor2 = abs(motor2.getPulses()); //Get number of pulses from motor2
        //des_vel2 = 20;
        mes_vel2 = (60*pulses_motor2) / (700 * dt * 4); // Motor Speed in RPM at the wheel
    
        if(pulses_motor2 == 0){
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
    
        if(mes_vel2 < 0){
            mes_vel2 = -mes_vel2; //Ensure Positive Feedback
        }
        e2 = r2 - mes_vel2; //Error, Ensure Negative Feedback
        D_Term2 = (e2 - e_Prev2)/dt; //Slope
    
        u2 = abs((e2*Kp + I_Term2*Ki + D_Term2*Kd));
    
        if(abs(u2) >= PWM_Period){
            u2 = PWM_Period;//Max Duty Cycle
        }
        else{
            if(des_vel2 != 0 && mes_vel2 != 0){
                I_Term2 = I_Term2 + e2*dt; //Previous error + error*time step, scaling factor 10e2
            }
        }
    
        PWM_motor2.pulsewidth_us(abs(u2));
        e_Prev2 = e2; //Previous Error
        motor2.reset(); //Reset the motor1 encoder 
    }
    
}

// ******** Encoder Interrupt Handler ********
void Encoder_ISR(void) {
    osSignalSet(EncoderID,0x1); // Send signal to the thread with ID, EncoderID, i.e., ExtThread.
}

// ******** Periodic Interrupt Handler 1********
void PeriodicInterruptISR(void){
    osSignalSet(PiControlId,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}

// ******** Periodic Interrupt Handler 2********
void PeriodicInterrupt2ISR(void){
    osSignalSet(PiControl2Id,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}

