#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
//#include "ROBOT_CONFIG.hpp"
#include "SbusParser.hpp"

#define _SBUS_EVENT_FLAG 0x10

//////////////////////////////////////////////////////////////
// ROBOT_CONFIG.hpp
//#define _MOAB_IP_ADDRESS "192.168.53.202"
#define _MOAB_IP_ADDRESS "192.168.11.20"
#define _NETMASK "255.255.255.0"
#define _DEFUALT_GATEWAY "192.168.11.1"
#define _BROADCAST_IP_ADDRESS "192.168.11.255"

#define _AUTOPILOT_IP_ADDRESS "192.168.11.40"


///////////////////////
// This is the "followbot" prototype that I use in AttracLab:
#define _STEERING_PW_CENTER 0.001424
#define _STEERING_PW_RANGE 0.000391
////////////////////////////////////////////////////////////

EventFlags event_flags;

uint16_t debug_port = 31337;
uint16_t sbus_port = 31338;

bool NETWORK_IS_UP = false;

// Network interface
EthernetInterface net;
UDPSocket rx_sock; // one, single thread for RX
UDPSocket tx_sock; // tx will be completely non-blocking

Thread udp_rx_thread;
Thread sbus_reTx_thread;

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);


// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PA_3, 100000);  // tx, then rx
Serial pc(USBTX,USBRX,115200);


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

uint16_t auto_ch1 = 1024;
uint16_t auto_ch2 = 1024;
void udp_rx_worker() {
	//
	 //Here we receive throttle and steering control from the auto-pilot computer
	 //
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
            pc.printf("auto_ch1 %d\n", auto_ch1);
            pc.printf("auto_ch2 %d\n", auto_ch2);

		} else if (n > 0) {
			inputBuffer[n] = 0;
			printf("rx %d bytes\n", n);
			printf(inputBuffer);
		} else {
			printf("empty packet\n");
		}
	}
}

struct sbus_udp_payload sbup;
SbusParser sbusParser(&sbup);

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

	while (true) {
		flags_read = event_flags.wait_any(_SBUS_EVENT_FLAG, 100);

		if (flags_read & osFlagsError) {
			//u_printf("S.Bus timeout!\n");
			set_mode_sbus_failsafe();
		} else if (sbup.failsafe) {
			//u_printf("S.Bus failsafe!\n");
			set_mode_sbus_failsafe();
		} else {
			if (sbup.ch5 < 688) {
				set_mode_stop();
			} else if (sbup.ch5 < 1360) {
				set_mode_manual();
			} else {
				set_mode_auto();
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

	//  ######################################
	//  #########################################
	//  ###########################################
	//   BEGIN:  setup network and udp socket
	//  ############################################

	printf("\n\nStarting the network...\n");

    const char *IP;

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
    IP = net.get_ip_address();
    pc.printf("ip: %X\n",IP);

	udp_rx_thread.start(udp_rx_worker);
	
	//  ############################################
	//   END:  setup network and udp socket
	//  ###########################################
	//  #########################################
	//  ######################################

	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);

	sbus_reTx_thread.start(sbus_reTx_worker);

	// Close the socket and bring down the network interface
	//rx_sock.close();
	//tx_sock.close();
	//net.disconnect();
	return 0;
}