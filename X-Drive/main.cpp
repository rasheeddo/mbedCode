/////////////////////////////////////////////////////////////
// This is the X-Drive (X-Wheels UGV) program on ATDrive MOAB
// SUBS read and parser were developed by Mark
// Brushless wheels drive was developed by Rasheed
/////////////////////////////////////////////////////////////

#include "mbed.h"
#include "rtos.h"
#include "SbusParser.hpp"
#include "XWheels.hpp"
#include "UbloxParser.hpp"
#include "Compass.hpp"
#include "EthernetInterface.h"
//#include "ROBOT_CONFIG.hpp"

#define _SBUS_EVENT_FLAG 0x10
/*
#define _GPS_EVENT_FLAG 0x20
#define _SHAFT_FALL_EVENT_FLAG 0x40
#define _PGM_FALL_EVENT_FLAG 0x80

/////////////////////////////// Variables /////////////////////////////// 
// PORT //
uint16_t debug_port = 31337;
uint16_t sbus_port = 31338;
uint16_t button_port = 31345;
uint16_t aux_serial_port_1 = 31341;
uint16_t gps_port = 27110;
uint16_t compass_port = 27111;
uint16_t odometry_port = 27112;
bool NETWORK_IS_UP = false;
*/
// AUTOPILOT //
uint16_t auto_ch2 = 400;        //1024
uint16_t auto_ch4 = 1024;       //1024
/*
// GPIN INTERRUPT SHAFT //
uint64_t _last_shaft_fall = 0;
uint64_t _last_shaft_rise = 0;

// INTERRUPT PGM //
volatile uint64_t _last_pgm_fall = 0;
volatile uint64_t _last_pgm_rise = 0;
uint64_t _last_pgm_fall_debounce = 0;
uint64_t _last_pgm_rise_debounce = 0;
uint8_t _pgm_value_debounce = 1;

// CHECH PGM BUTTON //
bool _pgm_notice_sent = false;
*/
// SBUS //
struct sbus_udp_payload sbup;

// DRIVE //
float motorRPM[2];

uint16_t ch2_map;
uint16_t ch4_map;

// GPR RX INTERRUPT //
//int gpsMessageLen;
//char _gpsMsgBuf[2048];

/////////////////////////////// Classes /////////////////////////////// 
// Network interface //
/*
EthernetInterface net;
UDPSocket rx_sock; // one, single thread for RX
UDPSocket aux_serial_sock; // one, single thread for RX
UDPSocket tx_sock; // tx will be completely non-blocking
*/

// THREAD //
//Thread udp_rx_thread;
Thread sbus_reTx_thread;
//Thread gps_reTx_thread;
//Thread compass_thread;
//Thread aux_serial_thread;
//Thread gp_interrupt_messages_thread;
EventFlags event_flags;

// MOTOR DRIVE //
XWheels drive;      // use Brushless wheels class for UGV

// SERIAL COM //
// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PA_3, 100000);  // tx, then rx
//RawSerial gps_in(PG_14, PG_9, 115200);  //tx, then rx
//RawSerial aux_serial1(PA_0, NC, 38400);
//InterruptIn shaft_encoder(PD_0, PullUp);
//InterruptIn pgm_switch(PD_1, PullUp);
Serial pc(USBTX,USBRX,115200);                              // for print out something to PC

// Heartbeat LED //
//PwmOut hb_led(PA_6);

// Compass //
//Compass compass(PF_15, PF_14); // sda, then scl

// Flight-mode LEDs //
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);

SbusParser sbusParser(&sbup);
UbloxParser ubloxParser;

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
void u_printf(const char *fmt, ...) {
	va_list args;
	char buffer[1500];
	int bufLen;

	va_start(args, fmt);
	bufLen = vsnprintf(buffer, 1499, fmt, args);
	va_end(args);

	int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, debug_port, buffer, bufLen);

	if (retval < 0 && NETWORK_IS_UP) {
		printf("socket error in u_printf() function\n");
		return;
	}
}
void udp_rx_worker() {
	
	//Here we receive throttle and steering control from the auto-pilot computer
	 
	SocketAddress sockAddr;
	char inputBuffer[33];
	inputBuffer[32] = 0;

	uint16_t *control = (uint16_t *) &(inputBuffer[0]);

	rx_sock.set_blocking(true);
	while (true) {

		int n = rx_sock.recvfrom(&sockAddr, inputBuffer, 32);

		if (n == 2*sizeof(uint16_t)) {
			auto_ch1 = control[0];
			auto_ch2 = control[1];
		} else if (n > 0) {
			inputBuffer[n] = 0;
			printf("rx %d bytes\n", n);
			printf(inputBuffer);
		} else {
			printf("empty packet\n");
		}
	}
}


void aux_serial_worker() {

	SocketAddress sockAddr;
	char inputBuffer[33];
	inputBuffer[32] = 0;

	aux_serial_sock.set_blocking(true);
	while (true) {

		int n = aux_serial_sock.recvfrom(&sockAddr, inputBuffer, 32);
		//u_printf("   for aux serial:  %s\n", inputBuffer);
		for (int i=0; i<n; ++i) {
			aux_serial1.putc(inputBuffer[i]);
		}
	}
}
*/

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
    drive.DriveWheels(0.0, 0.0);
}

void set_mode_manual() {
	myledR = 0;
	myledG = 1;
	myledB = 0;
	//compass.set_leds(0, 15, 0);
    ch2_map = map(sbup.ch2,368,1680,100,-100);       // map raw PWM value to understandable scale like -100 to 100 , 0 is mid 
    ch4_map = map(sbup.ch4,368,1680,-100,100);
    drive.vehicleControl(ch2_map, ch4_map, motorRPM);
    drive.DriveWheels(motorRPM[0],motorRPM[1]);

}

void set_mode_auto() {
	myledR = 0;
	myledG = 0;
	myledB = 1;
	//compass.set_leds(0, 0, 15);
    ch2_map = map(auto_ch2,368,1680,100,-100);       // map raw PWM value to understandable scale like -100 to 100 , 0 is mid 
    ch4_map = map(auto_ch4,368,1680,-100,100);
    drive.vehicleControl(ch2_map, ch4_map, motorRPM);
    drive.DriveWheels(motorRPM[0],motorRPM[1]);
}

/*
void Gpin_Interrupt_Shaft() {

	uint64_t ts = rtos::Kernel::get_ms_count();
	//if (ts - _last_shaft_fall > 25) {
		_last_shaft_fall = ts;
		event_flags.set(_SHAFT_FALL_EVENT_FLAG);
	//}
}

void Gpin_Interrupt_Pgm() {

	if (pgm_switch.read()) {
		_last_pgm_rise = rtos::Kernel::get_ms_count();
	} else {
		_last_pgm_fall = rtos::Kernel::get_ms_count();
	}
}

void Check_Pgm_Button() {
	// We do all this time-stamp weirdness to de-bounce the switch
	uint64_t ts_ms = rtos::Kernel::get_ms_count();

	// Switch must be stable for at least 30ms:
	if ((ts_ms - _last_pgm_fall > 30) && (ts_ms - _last_pgm_rise > 30)) {
		// current stable value:
		uint8_t val = pgm_switch.read();
		if (_pgm_value_debounce != val) { // debounce value has changed
			_pgm_value_debounce = val;
			if (val) {
				_last_pgm_rise_debounce = ts_ms;
				_pgm_notice_sent = false;
			} else {
				_last_pgm_fall_debounce = ts_ms;
			}
		}
	}

	if (!_pgm_notice_sent) {
		if (!_pgm_value_debounce) {
			if (ts_ms - _last_pgm_rise_debounce > 300) {

				int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, button_port,
					&ts_ms, sizeof(ts_ms));

				if (retval < 0 && NETWORK_IS_UP) {
					printf("UDP socket error in Gpio_Tx_worker\n");
				}
				_pgm_notice_sent = true;
			}
		}
	}
}


void Gpio_Tx_Worker() {
	uint32_t flags_read;
	while (true) {
		flags_read = event_flags.wait_any(_SHAFT_FALL_EVENT_FLAG | _PGM_FALL_EVENT_FLAG, 1013);

		if (flags_read & osFlagsError) {

			//u_printf("GPIn Interrupt timeout!\n");

		} else {
			if (flags_read & _SHAFT_FALL_EVENT_FLAG) {
				//u_printf("  SHAFT ENCDODER\n");
				int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, odometry_port,
					&_last_shaft_fall, sizeof(_last_shaft_fall));

				if (retval < 0 && NETWORK_IS_UP) {
					printf("UDP socket error in Gpio_Tx_worker\n");
				}
			}
    
		//	if (flags_read & _PGM_FALL_EVENT_FLAG) {
		//		u_printf(" *********************** BUTTON! ****************\n");
		//	}
    
		}
	}
}
*/

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
/*
void Gps_Rx_Interrupt() {
	int c;
	while (gps_in.readable()) {

		c = gps_in.getc();
		int status = ubloxParser.rx_char(c);
		//gpsMessageLen = ubloxParser.rx_char(c);

		if (status > 0) {
			memcpy(_gpsMsgBuf, ubloxParser._rxBuf, status);
			gpsMessageLen = status;
			event_flags.set(_GPS_EVENT_FLAG);
		}
	}
}

void gps_reTx_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = event_flags.wait_any(_GPS_EVENT_FLAG, 500);

		if (flags_read & osFlagsError) {

			u_printf("GPS timeout!\n");

		} else if (gpsMessageLen>8 && gpsMessageLen <2000) {

			int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, gps_port,
					_gpsMsgBuf, gpsMessageLen);

			if (retval < 0 && NETWORK_IS_UP) {
				printf("UDP socket error in gps_reTx_worker\n");
			}
		} else {
			u_printf("gps msglen: %d\n", gpsMessageLen);
		}
	}
}
*/

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
/*
void compass_worker() {

	int16_t compass_XYZ[3];

	while (true) {
		if (compass.get_data(compass_XYZ)) {

			int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, compass_port,
					(char *) compass_XYZ, 6);

			// This could produce a lot of messages...
			//if (retval < 0 && NETWORK_IS_UP) {
			//	printf("UDP socket error in sbus_reTx_worker\n");
			//}
		}

		wait(0.05); // 20Hz
	}
}


void eth_callback(nsapi_event_t status, intptr_t param) {
	const char *ip;

	printf("Connection status changed!\r\n");
	switch(param) {
		case NSAPI_STATUS_LOCAL_UP:
			NETWORK_IS_UP = false;
			printf("Local IP address set.\r\n");
			break;
		case NSAPI_STATUS_GLOBAL_UP:
			printf("Global IP address set.\r\n");
			NETWORK_IS_UP = true;
			ip = net.get_ip_address();  // <--dhcp
			if (ip) {
				printf("IP address is: %s\n", ip);
			} else {
				printf("no IP address... we're screwed\n");
				//return -1;
			}
			break;
		case NSAPI_STATUS_DISCONNECTED:
			NETWORK_IS_UP = false;
			printf("No connection to network.\r\n");
			break;
		case NSAPI_STATUS_CONNECTING:
			NETWORK_IS_UP = false;
			printf("Connecting to network...\r\n");
			break;
		default:
			NETWORK_IS_UP = false;
			printf("Not supported");
			break;
	}
}
*/

int main() {
    /*
	//  ######################################
	//  #########################################
	//  ###########################################
	//   BEGIN:  setup network and udp socket
	//  ############################################

	printf("\n\nStarting the network...\n");

	net.attach(&eth_callback);
	net.set_dhcp(false);
	net.set_network(_MOAB_IP_ADDRESS, _NETMASK, _DEFUALT_GATEWAY);
	net.set_blocking(false);

	rx_sock.open(&net);
	rx_sock.bind(12346);

	tx_sock.open(&net);
	tx_sock.bind(12347);
	tx_sock.set_blocking(false);

	aux_serial_sock.open(&net);
	aux_serial_sock.bind(31341);

	net.connect();

	udp_rx_thread.start(udp_rx_worker);
	aux_serial_thread.start(aux_serial_worker);

	//  ############################################
	//   END:  setup network and udp socket
	//  ###########################################
	//  #########################################
	//  ######################################
    */
    
	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);
    //gps_in.attach(&Gps_Rx_Interrupt);

	sbus_reTx_thread.start(sbus_reTx_worker);
    /*
    gps_reTx_thread.start(gps_reTx_worker);
	compass_thread.start(compass_worker);
	gp_interrupt_messages_thread.start(Gpio_Tx_Worker);

	//shaft_encoder.rise(&Gpin_Interrupt_Shaft);
	shaft_encoder.fall(&Gpin_Interrupt_Shaft);

	pgm_switch.rise(&Gpin_Interrupt_Pgm);
	pgm_switch.fall(&Gpin_Interrupt_Pgm);

	hb_led.period(0.02);
	hb_led.write(0.0);


	// Look for the compass:
	if (compass.init() < 1) {
		u_printf("Failed to initialize compass\n");
	}


	for (int ct=0; true; ++ct){

		for (int i=0; i < 11; ++i) {
				float brightness = i/10.0;
				hb_led.write(brightness);
				//motorControl.trigger_pw_out();
				wait(0.02);
				Check_Pgm_Button();
		}
		for (int i=0; i < 11; ++i) {
				float brightness = 1.0 - i/10.0;
				hb_led.write(brightness);
				//motorControl.trigger_pw_out();
				wait(0.02);
				Check_Pgm_Button();
		}

		u_printf("heeartbeatZ: %d\n", ct);

		// Report motor values (for convience when setting trim)
		uint16_t sbus_a = motorControl.get_value_a();
		float pw_a = motorControl.get_pw_a();
		u_printf("steering: %d %f\n", sbus_a, pw_a);

		uint16_t sbus_b = motorControl.get_value_b();
		float pw_b = motorControl.get_pw_b();
		u_printf("throttle: %d %f\n", sbus_b, pw_b);
	}

   
	// Close the socket and bring down the network interface
	rx_sock.close();
	tx_sock.close();
	net.disconnect();
    */

	return 0;
}