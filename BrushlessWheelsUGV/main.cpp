#include "mbed.h"
#include "rtos.h"
#include "SbusParser.hpp"
#include "BrushlessWheels.hpp"

#define _SBUS_EVENT_FLAG 0x10

EventFlags event_flags;
Thread sbus_reTx_thread;

// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PA_3, 100000);  // tx, then rx
Serial pc(USBTX,USBRX);

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);


struct sbus_udp_payload sbup;
SbusParser sbusParser(&sbup);

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
        pc.printf("flags_read %X\n", flags_read);
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
				set_mode_stop();
			} else if (sbup.ch5 < 1360) {
				set_mode_manual();
			} else {
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

int main() {

    pc.baud(115200);

	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);

	sbus_reTx_thread.start(sbus_reTx_worker);

    

	return 0;
}


