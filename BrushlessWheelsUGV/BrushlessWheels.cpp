
#include "BrushlessWheels.hpp"
#include "mbed.h"

RawSerial uart(PD_1,PD_0,9600);
//Serial pc(USBTX,USBRX,115200);

BrushlessWheels::BrushlessWheels()
{
    
    startTick = true;
    ReadOK = false;
    Reply[0] = 1;
    Reply[1] = 1;
    Reply[2] = 1;
    Reply[3] = 1;
    Reply[4] = 1;
    Reply[5] = 1;
    Header1 = 0x02;
    Header2 = 0x09;
}

int BrushlessWheels::Init()
{
    //pc.printf("Initialized OK...\n");
    waitUntilFourZero();
    wait_ms(219);
    ESCHandShake();
    for(i=1;i<10;i++)
    {
        zeroSpeed();
    }
    return 1;
}

void BrushlessWheels::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2])
{
  // unsigned int can store 16 bit int 
  StoreByte[0] = (Data & 0xFF00) >> 8;                  //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF);                       //Low byte, most left of HEX
}

long BrushlessWheels::map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned int BrushlessWheels::RPMToRaw(float rpm)
{
  int raw_int;
  unsigned int out_raw;

    // map rpm to raw value
    raw_int = (int)map(rpm, 0.0, 144.0, 0, 3200);
  
    // In case of negative number, shift mapped number from 32768 to 35968 (for 0.0 to -146.0)
    if (rpm < 0.0)
    {
      out_raw = 32768 + abs(raw_int);
      }
    // In case of positive number, use the mapped number for unsigned int type
    else
    {
      out_raw = raw_int;
      }
  
  return out_raw;
}
void BrushlessWheels::waitUntilFourZero()
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
            
            if ((Reply[0] == 0) && (Reply[1] == 0) && (Reply[2] == 0) && (Reply[3] == 0)){
                startTick = false;
                //pc.printf("Ready to drive...\n");
            }
            
            // reset value
            i = 0;
            ReadOK = false;

            }
            wait_ms(1);    // wait_ms(15); DONT CHANGE THIS DELAY   This delay act as "pc.printf("Readable : %d\n",uart.readable());"
                           // wait_ms(1) mimic all of the printf behavior and it works
    }
}

void BrushlessWheels::ESCHandShake()
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

void BrushlessWheels::zeroSpeed()
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

void BrushlessWheels::DriveWheels(float rpm1, float rpm2)
{

    RawInt1 = RPMToRaw(rpm1);
    RawInt2 = RPMToRaw(rpm2);
    Int16ToByteData(RawInt1,Motor1SpeedByte);
    Int16ToByteData(RawInt2,Motor2SpeedByte);

    unsigned char Motor1hibyte = Motor1SpeedByte[0];
    unsigned char Motor1lobyte = Motor1SpeedByte[1];

    unsigned char Motor2hibyte = Motor2SpeedByte[0];
    unsigned char Motor2lobyte = Motor2SpeedByte[1];

    unsigned char Modehibyte = 0x00;    // don't care 
    unsigned char Modelobyte = 0x00;    // don't care
    unsigned char CheckSum = Header1 + Header2 + Motor1hibyte + Motor1lobyte + Motor2hibyte + Motor2lobyte + Modehibyte + Modelobyte;

    uart.putc(Header1);
    uart.putc(Header2);
    uart.putc(Motor1hibyte);
    uart.putc(Motor1lobyte);
    uart.putc(Motor2hibyte);
    uart.putc(Motor2lobyte);
    uart.putc(Modehibyte);
    uart.putc(Modelobyte);
    uart.putc(CheckSum);

    wait_ms(23);                      // DON'T change this delay, it's from hacking

  
}