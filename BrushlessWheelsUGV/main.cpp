// SUBS read and parser were developed by Mark
// Brushless wheels drive was developed by Rasheed

#include "mbed.h"
#include "rtos.h"
#include "SbusParser.hpp"
#include "BrushlessWheels.hpp"
#include <Semaphore.h>

#define _SBUS_EVENT_FLAG 0x10

// VEHICLE CONTROL , UGV DRIVE
#define MAX_RPM 80         // Max RPM of the wheels, this is limited by wheels itself. Default is 144
#define ZERO_RPM 0          // No speed
#define MIN_STICK 10        // after mapped
#define MAX_STICK 100       // after mapped
#define DIVIDER 2           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

EventFlags event_flags;
Thread sbus_reTx_thread;
Thread drive_thread;        // a thread for drive UGV

Semaphore sem(1);           // use Semaphore to access sbup variable from another thread 
int ch2_share;              // a global variable to copy sbup.ch2 and paste in ch_share
int ch4_share;

BrushlessWheels drive;      // use Brushless wheels class for UGV

// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PA_3, 100000);  // tx, then rx
Serial pc(USBTX,USBRX,115200);      // for print out something to PC

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);


struct sbus_udp_payload sbup;
SbusParser sbusParser(&sbup);

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Sbus_Rx_Interrupt() {

	int c;

	while (sbus_in.readable()) {

		c = sbus_in.getc();
		int status = sbusParser.rx_char(c);

		if (status == 1) {
			event_flags.set(_SBUS_EVENT_FLAG);
		}
	}
}

void set_mode_sbus_failsafe() {
	myledR = 0;
	myledG = 0;
	myledB = 0;
	//compass.set_leds(0, 0, 0);
	//motorControl.set_steering(1024);
	//motorControl.set_throttle(352);
}

void set_mode_stop() {
	myledR = 1;
	myledG = 0;
	myledB = 0;
	//compass.set_leds(15, 0, 0);
	//motorControl.set_steering(1024);
	//motorControl.set_throttle(352);
}

void set_mode_manual() {
	myledR = 0;
	myledG = 1;
	myledB = 0;
	//compass.set_leds(0, 15, 0);
	//motorControl.set_steering(sbup.ch1);
	//motorControl.set_throttle(sbup.ch3);
}

void set_mode_auto() {
	myledR = 0;
	myledG = 0;
	myledB = 1;
	//compass.set_leds(0, 0, 15);
	//motorControl.set_steering(auto_ch1);
	//motorControl.set_throttle(auto_ch2);
}

void sbus_reTx_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = event_flags.wait_any(_SBUS_EVENT_FLAG, 100);
        //pc.printf("flags_read %X\n", flags_read);
        //pc.printf("Here");

		if (flags_read & osFlagsError) {
			printf("S.Bus timeout!\n");
			set_mode_sbus_failsafe();
		} else if (sbup.failsafe) {
			printf("S.Bus failsafe!\n");
			set_mode_sbus_failsafe();
		} else {

            //pc.printf("ch2 %d\n",sbup.ch2);
            //pc.printf("ch4 %d\n",sbup.ch4);

			if (sbup.ch5 < 688) {
                sem.wait(0);
                ch2_share = 1016;
                ch4_share = 1032;
                sem.release();
                set_mode_stop();

			} else if (sbup.ch5 < 1360) {
                sem.wait(0);
                ch2_share = sbup.ch2;
                ch4_share = sbup.ch4;
                sem.release();
                set_mode_manual();
			} else {
                sem.wait(0);
                ch2_share = 368;
                ch4_share = 1032; 
                sem.release();
				set_mode_auto();
			}
			/*
			int retval = tx_sock.sendto(_AUTOPILOT_IP_ADDRESS, sbus_port,
					(char *) &sbup, sizeof(struct sbus_udp_payload));

			if (retval < 0 && NETWORK_IS_UP) {
				printf("UDP socket error in sbus_reTx_worker\n");
			}
			*/
		}
	}
}


void vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2])
{   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MIN_STICK is 10, a minimum value of stick after mapped from PWM (from map it was 0, but some channel it's not 0, so set it to 10 as minimum)
    // MAX_STICK is 100, a maximum value of stick after mapped from PWM
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

    /////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (LR_ch < MIN_STICK && LR_ch > -MIN_STICK && UD_ch < MIN_STICK && UD_ch > -MIN_STICK)
    {
        MotorRPM[0] = 0.0;
        MotorRPM[1] = 0.0;
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(LR_ch <= MIN_STICK && LR_ch >= -MIN_STICK && (UD_ch >= MIN_STICK || UD_ch <= -MIN_STICK))
    {
        MotorRPM[0] = (float)map(UD_ch, -MAX_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[1] = MotorRPM[0];

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MIN_STICK && UD_ch >= -MIN_STICK && (LR_ch >= MIN_STICK || LR_ch <= -MIN_STICK))
    {
        MotorRPM[1] = (float)map(LR_ch, -MAX_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[0] = -MotorRPM[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch >= MIN_STICK && LR_ch >= MIN_STICK)
    {
        MotorRPM[1] = (float)map(UD_ch, MIN_STICK, MAX_STICK, ZERO_RPM, MAX_RPM);
        MotorRPM[0] = MotorRPM[1]/DIVIDER;
    }

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch >= MIN_STICK && LR_ch <= -MIN_STICK)
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_STICK, MAX_STICK, ZERO_RPM, MAX_RPM);
        MotorRPM[1] = MotorRPM[0]/DIVIDER;
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch <= -MIN_STICK && LR_ch <= -MIN_STICK)
    {
        MotorRPM[0] = (float)map(UD_ch, -MIN_STICK, -MAX_STICK, ZERO_RPM, -MAX_RPM);
        MotorRPM[1] = MotorRPM[0]/DIVIDER;
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch <= -MIN_STICK && LR_ch >= MIN_STICK)
    {
        MotorRPM[1] = (float)map(UD_ch, -MIN_STICK, -MAX_STICK, ZERO_RPM, -MAX_RPM);
        MotorRPM[0] = MotorRPM[1]/DIVIDER;
    }  
   
}


void UGV_drive()
{

    int ch2_get;
    int ch4_get;

    int ch2_map;
    int ch4_map;
    float motorRPM[2];

    while (true){

        sem.wait(0);        
        ch2_get = ch2_share;
        ch4_get = ch4_share;
        sem.release();
        //pc.printf("ch2_get: %d\n", ch2_get);
        //pc.printf("ch4_get: %d\n", ch4_get);
        ch2_map = map(ch2_get,368,1680,100,-100);       // map raw PWM value to understandable scale like -100 to 100 , 0 is mid 
        ch4_map = map(ch4_get,368,1680,-100,100);
        // set a zero span of less than +-3
        if (abs(ch2_map)<3)
        {
            ch2_map = 0;
        }
        if (abs(ch4_map)<3)
        {
            ch4_map = 0;
        }
        //pc.printf("ch2_map: %d\n", ch2_map);
        //pc.printf("ch4_map: %d\n", ch4_map);

        vehicleControl(ch2_map, ch4_map, motorRPM);     // take ch2 and ch4 value and decide how fast each wheel needs

        drive.DriveWheels(motorRPM[0], motorRPM[1]);    // use that calculated speed drive the wheels in BrushlesWheels class
        
    }
    

}


int main() {

    int initOK;
    initOK = drive.Init();
    if(initOK == 1)
    {
        pc.printf("Initialized OK!!!\n");
    }
    

	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);

	sbus_reTx_thread.start(sbus_reTx_worker);
    drive_thread.start(UGV_drive);

    

	return 0;
}


