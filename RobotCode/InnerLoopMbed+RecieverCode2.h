/*
General Information:
 - The lowest speed the motors can rotate is approximately 15 RPM at the wheel
 - Motor 1 is the right motor, direction of 0 makes it go forward and 1 makes it reverse
 - Motor 2 is the left motor, direction of 0 makes it go forward and 1 makes it reverse
 
Ports In Use:
 - PTE4 Motor1 Direction
 - PTE5 Motor2 Direction
 - PTD1,PTD3 Encoder Motor1
 - PTD2,PTD0 Encoder Motor2
 - PTE20 PWM Motor1
 - PTE22 PWM Motor2
*/

#include "mbed.h"
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
void get_Transmitted_Data();

//Serial
Serial pc(USBTX, USBRX); // Pins (tx, rx) for PC serial channel

//Processes and Threads
osThreadId EncoderID, Encoder2ID, PiControlId, PiControl2Id; // Thread ID's
osThreadDef(PID1, osPriorityRealtime, 1024); // Declare PiControlThread as a thread, highest priority
osThreadDef(PID2, osPriorityRealtime, 1024); // Declare PiControlThread2 as a thread, highest priority
osThreadDef(EncoderThread, osPriorityRealtime, 1024); // Declare PiControlThread as a thread, highest priority

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


#define TransferSize    8
// Data Packets Requirements
#define StartBit        0xFF
#define IDbit           0x01
#define EndBit          0xFF
// data array containing the received packet from the UART 
uint8_t Received[TransferSize];


int main(){
    pc.baud(256000);  // Set max uart baud rate
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
    
    char test = 0;
    //Loop Forever
    while(true) {

        if(pc.readable()){
            get_Transmitted_Data();
            //pc.putc(pc.getc());
        }
            
            /*
            dir_motor2 = pc.getc(); //Get Left Motor Direction
            des_vel2 = pc.getc(); //Get Left Motor Speed
            dir_motor1 = pc.getc(); //Get Right Motor Direction
            des_vel = pc.getc(); //Get Right Motor Speed
            */
        
       
        /*
        // Printing Encoder Data
        // pc.printf("\n\r Velocity Motor 1: ");
        pc.printf("%f ", mes_vel); // Speed
        pc.printf(",");
        pc.printf("%f ", e);    //  #error
        pc.printf(",");
        pc.printf("%f ", u);    //  #error
        pc.printf(",");
        pc.printf("%f ", mes_vel2);    // integrator state
        pc.printf(",");
        pc.printf("%f ", e2);   // control input  
        pc.printf(","); 
        pc.printf("%f ", u2);    //  #error
        pc.printf(","); 
        pc.printf("%f", pulses_motor1);
        pc.printf(","); 
        pc.printf("%f", pulses_motor2);
        pc.printf("\n\r");
        */
        //osDelay(50); // Go to sleep on the waiting queue for 50 ms
        
    }

}

void get_Transmitted_Data(){
    int i=0;
    
    for(i=0; i < TransferSize; i++){
        Received[i] = pc.getc();
        pc.putc(Received[i]);
    }
    if((Received[0] == StartBit) &&  (Received[1] == IDbit) &&  (Received[7] == EndBit)){
        // Transmitted Data
        des_vel = Received[2];
        dir_motor1=Received[3];
        des_vel2 = Received[4];
        dir_motor2=Received[5];
        kick  = Received[6]; 
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
