
#include "BrushlessWheels.hpp"

BrushlessWheels::BrushlessWheels()
{
     
    
}

void BrushlessWheels::Init()
{
    RawSerial uart(PD_1,PD_0);  //Tx4, Rx4
    waitUnitlFourZero();
}

void waitUntilFourZero()
{
    while (startTick)
    {
        //t.start();
        //pc.printf("Readable : %d\n",uart.readable());
        //t.stop();
        //pc.printf("Time: %f seconds\n", t.read());
        while (uart.readable() == true) {
            char ReadByte = uart.getc();
            Reply[i] = ReadByte;
            //t.start();
            //pc.printf("ReadByte: %X\n", ReadByte);     // print whole string
            //t.stop();
            //pc.printf("Time: %f seconds\n", t.read());
            i++;
            ReadOK = true;
            wait_us(1000);  // DONT CHANGE THIS DELAY   This delay act as "pc.printf("ReadByte: %X\n", ReadByte);"
            }

        if (ReadOK == true){
            
            //t.start();
            pc.printf("i:%d \n",i);
            //t.stop();
            //pc.printf("Time: %f seconds\n", t.read());
            
           for(int j=0;j<i;j++){
                pc.printf("j%d\n",j);
                pc.printf("Reply: %X\n", Reply[j]);     // print whole string
            }
            
            if ((Reply[0] == 0) && (Reply[1] == 0) && (Reply[2] == 0) && (Reply[3] == 0)){
                pc.printf("startTick false\n");
                startTick = false;
            }
            
            // reset value
            i = 0;
            ReadOK = false;

            }

            wait_ms(15);    // wait_ms(15); DONT CHANGE THIS DELAY   This delay act as "pc.printf("Readable : %d\n",uart.readable());"
    }
}

void ESCShakeHand()
{
    for(int k=1;k<=20;k++)
    {
        uart.putc(0x01);
        uart.putc(0x11);
        uart.putc(0x28);
        uart.putc(0x02);
        uart.putc(0x50);
        uart.putc(0x32);
        uart.putc(0x03);
        uart.putc(0x1E);
        uart.putc(0x83);
        uart.putc(0x15);
        uart.putc(0x06);
        uart.putc(0x0A);
        uart.putc(0x01);
        uart.putc(0x03);
        uart.putc(0x04);
        uart.putc(0x07);
        uart.putc(0x96);

        if (k==1){
            wait_us(300);
        }
        else{
            wait_ms(14);
            
        }
    }
}

void zeroSpeed()
{
    uart.putc(Header1);
    uart.putc(Header2);
    uart.putc(0x00);            // Motor1 speed hibyte
    uart.putc(0x00);            // Motor1 speed lobyte
    uart.putc(0x00);            // Motor2 speed hibyte
    uart.putc(0x00);            // Motor2 speed lobyte
    uart.putc(0xB4);            // Mode hibyte (don't care)
    uart.putc(0x00);            // Mode lobyte (don't care)
    uart.putc(0xBF);            // Check sum
    wait_ms(23);

}
