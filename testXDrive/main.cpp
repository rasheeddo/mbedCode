#include "mbed.h"
#include "XWheels.hpp"

RawSerial wheelUART(PD_1,PD_0,9600);
RawSerial pc(USBTX,USBRX,115200);
XWheels drive(&wheelUART);

// main() runs in its own thread in the OS
int main()
{

    // X Drive Initialize ///
    int initOK;
    initOK = drive.Init();
    if(initOK == 1)
    {
        pc.printf("Initialized OK!!!\n");
    }
    while (true){
        drive.DriveWheels(60.0, 60.0);
    }
    
}
