#include "mbed.h"
#include "EthernetInterface.h"

// Network interface
EthernetInterface net;
Serial pc(USBTX,USBRX,115200);

// Socket demo
int main() {
    // Bring up the ethernet interface
    printf("Ethernet socket example\n");
    net.connect();

    // Show the network address
    const char *ip = net.get_ip_address();
    pc.printf("IP address is: %s\n", ip ? ip : "No IP");

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    //TCPSocket socket;
    //socket.open(&net);
    //socket.connect("www.arm.com", 80);

    // Send a simple http request
    //char sbuffer[] = "GET / HTTP/1.1\r\nHost: www.arm.com\r\n\r\n";
    //int scount = socket.send(sbuffer, sizeof sbuffer);
    //pc.printf("sent %d [%.*s]\n", scount, strstr(sbuffer, "\r\n")-sbuffer, sbuffer);

    // Recieve a simple http response and print out the response line
    //char rbuffer[64];
    //int rcount = socket.recv(rbuffer, sizeof rbuffer);
    //pc.printf("recv %d [%.*s]\n", rcount, strstr(rbuffer, "\r\n")-rbuffer, rbuffer);

    // Close the socket to return its memory and bring down the network interface
    //socket.close();

    // Bring down the ethernet interface
    //net.disconnect();
    pc.printf("Done\n");
}