/////////////////////////////////////////////////////////////
// This is the X-Drive (X-Wheels UGV) program on ATDrive MOAB
// SUBS read and parser were developed by Mark
// Brushless wheels drive was developed by Rasheed
/////////////////////////////////////////////////////////////

#include "mbed.h"
#include "rtos.h"
#include "SbusParser.hpp"
#include "XWheels.hpp"
//#include "UbloxParser.hpp"
#include "Compass.hpp"
#include "EthernetInterface.h"
//#include "ROBOT_CONFIG.hpp"
//////////////////////////////////////////////////////////////
// ROBOT_CONFIG //
// Office LAN use
#include "ROBOT_CONFIG_XDRIVE_OfficeLAN.hpp"
// Field LAN Use
//#include "ROBOT_CONFIG_XDRIVE_GLiNet.hpp"
///////////////////////

// This is the "followbot" prototype that I use in AttracLab:
#define _STEERING_PW_CENTER 0.001424
#define _STEERING_PW_RANGE 0.000391
////////////////////////////////////////////////////////////

#define _SBUS_EVENT_FLAG 0x10

#define _GPS_EVENT_FLAG 0x20
#define _SHAFT_FALL_EVENT_FLAG 0x40
#define _PGM_FALL_EVENT_FLAG 0x80

/////////////////////////////// Variables /////////////////////////////// 
// PORT //
uint16_t debug_port = 31337;
uint16_t sbus_port = 31338;
uint16_t button_port = 31345;
uint16_t aux_serial_port_1 = 31341;
//uint16_t gps_port = 27110;
uint16_t gps_port_nmea = 27113; // NMEA 27113
uint16_t compass_port = 27111;
uint16_t odometry_port = 27112;
bool NETWORK_IS_UP = false;

// AUTOPILOT //
uint16_t auto_ch2 = 1024;        //1024
uint16_t auto_ch4 = 1024;       //1024
float rpmR;
float rpmL;
uint16_t Raw_rpmR;
uint16_t Raw_rpmL;

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

// SBUS //
struct sbus_udp_payload sbup;

// DRIVE //
float motorRPM[2];

int ch2_map;
int ch4_map;

// GPS RX INTERRUPT //
// Incoming buffer, from serial port:
char _gpsRxBuf[1500];
int _gpsRxBufIdx = 0;

// Outgoing buffer, via network UDP:
char _gpsTxBuf[1500];
int gpsMessageLen;

/////////////////////////////// Classes /////////////////////////////// 
// Network interface //
 
EthernetInterface net;
UDPSocket rx_sock; // one, single thread for RX
UDPSocket aux_serial_sock; // one, single thread for RX
UDPSocket tx_sock; // tx will be completely non-blocking


// THREAD //
Thread udp_rx_thread;
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
RawSerial sbus_in(NC, PD_2, 100000);  // tx, then rx
//RawSerial sbus_in(NC, PA_3, 100000);  // tx, then rx
//RawSerial gps_in(PE_8, PE_7, 38400);  //tx, then rx
//RawSerial gps_in(PG_14, PG_9, 115200);  // tx, then rx
//RawSerial aux_serial1(PA_0, NC, 38400);
//InterruptIn shaft_encoder(PD_0, PullUp);
//InterruptIn pgm_switch(PD_1, PullUp);
Serial pc(USBTX,USBRX,115200);                              // for print out something to PC

// Heartbeat LED //
PwmOut hb_led(PA_6);

// Compass //
Compass compass(PB_11, PB_10); // sda, then scl
//Compass compass(PF_15, PF_14); // sda, then scl

// Flight-mode LEDs //
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);

SbusParser sbusParser(&sbup);
//UbloxParser ubloxParser;

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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

	//uint16_t *control = (uint16_t *) &(inputBuffer[0]);
    float *control = (float *) &(inputBuffer[0]);

	rx_sock.set_blocking(true);
	while (true) {

		int n = rx_sock.recvfrom(&sockAddr, inputBuffer, 32);
           //(n == 2*sizeof(uint16_t))
		if (n == 2*sizeof(float)) {
			//auto_ch2 = control[0];
			//auto_ch4 = control[1];
            //Raw_rpmR = control[0];
            //Raw_rpmL = control[1];
            rpmR = control[0];
            rpmL = control[1];

		} else if (n > 0) {
			inputBuffer[n] = 0;
			printf("rx %d bytes\n", n);
			printf(inputBuffer);
		} else {
			printf("empty packet\n");
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
    drive.DriveWheels(0.0, 0.0);
}

void set_mode_manual() {
	myledR = 0;
	myledG = 1;
	myledB = 0;
	
    pc.printf("ch3 %d\n", sbup.ch3);
    pc.printf("ch2 %d\n", sbup.ch2);
    drive.vehicleControl(sbup.ch3, sbup.ch2, motorRPM);
    //pc.printf("rpm1 %f\n", motorRPM[0]);
    //pc.printf("rpm2 %f\n", motorRPM[1]);
    drive.DriveWheels(motorRPM[0],motorRPM[1]);
    
}

void set_mode_auto() {
	myledR = 0;
	myledG = 0;
	myledB = 1;
	
    drive.DriveWheels(rpmR,rpmL);
    printf("rpmR %f\n", rpmR);
    printf("rpmL %f\n", rpmL);
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

void sbus_reTx_worker() {

	uint32_t flags_read;
    bool stop_trig = false;
    printf("sbus worker start\n");
	while (true) {
		flags_read = event_flags.wait_any(_SBUS_EVENT_FLAG, 100);

		if (flags_read & osFlagsError) {
			u_printf("S.Bus timeout!\n");
            pc.printf("S.Bus timeout!\n");
			set_mode_sbus_failsafe();
		} else if (sbup.failsafe) {
			u_printf("S.Bus failsafe!\n");

        
            pc.printf("S.Bus failsafe!\n");
			set_mode_sbus_failsafe();
		} else {
            /*
            pc.printf("ch1 %d   ",sbup.ch1);
            pc.printf("ch2 %d   ",sbup.ch2);
            pc.printf("ch3 %d   ",sbup.ch3);
            pc.printf("ch4 %d   ",sbup.ch4);
            pc.printf("ch5 %d   ",sbup.ch5);
            pc.printf("ch6 %d   ",sbup.ch6);
            pc.printf("ch7 %d   ",sbup.ch7);
            pc.printf("ch8 %d\n",sbup.ch8);
            */
            if (sbup.ch7 < 1050 && sbup.ch7 > 950 && sbup.ch8 < 1050 && sbup.ch8 > 950 && sbup.ch6 < 1500 && !stop_trig) {

                set_mode_manual();
                pc.printf("manual\n");

			} else if (sbup.ch7 > 1050 && sbup.ch7 < 1100 && sbup.ch8 > 1050 && sbup.ch8 < 1100 && sbup.ch6 < 1500 && !stop_trig){

				set_mode_auto();
                pc.printf("auto\n");

			} else {
                pc.printf("stop\n");
                set_mode_stop();
                if (sbup.ch5 > 1500){
                    stop_trig = false;
                } else {
                    stop_trig = true;
                }
            }
			
			int retval = tx_sock.sendto(_AUTOPILOT_IP_ADDRESS, sbus_port,
					(char *) &sbup, sizeof(struct sbus_udp_payload));

			if (retval < 0 && NETWORK_IS_UP) {
				printf("UDP socket error in sbus_reTx_worker\n");
			}
			
		}
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


int main() {

        /// X Drive Initialize ///
    int initOK;
    initOK = drive.Init();
    if(initOK == 1)
    {
        pc.printf("Initialized OK!!!\n");
    }

    sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);
	sbus_reTx_thread.start(sbus_reTx_worker);
    ////////////////////////////
    
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

	net.connect();

	udp_rx_thread.start(udp_rx_worker);

	//  ############################################
	//   END:  setup network and udp socket
	//  ###########################################
	//  #########################################
	//  ######################################
    

	hb_led.period(0.02);
	hb_led.write(0.0);

	for (int ct=0; true; ++ct){

		for (int i=0; i < 11; ++i) {
				float brightness = i/10.0;
				hb_led.write(brightness);
				wait(0.02);

		}
		for (int i=0; i < 11; ++i) {
				float brightness = 1.0 - i/10.0;
				hb_led.write(brightness);
				wait(0.02);
		}

		u_printf("heeartbeatZ: %d\n", ct);
	}
    
	// Close the socket and bring down the network interface
	
    rx_sock.close();
	tx_sock.close();
	net.disconnect();
    

	return 0;
}