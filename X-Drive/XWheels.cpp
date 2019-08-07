
#include "XWheels.hpp"
#include "mbed.h"
// VEHICLE CONTROL , UGV DRIVE
#define MAX_RPM 144         // Max RPM of the wheels, this is limited by wheels itself. Default is 144
#define ZERO_RPM 0          // No speed
#define MIN_STICK 360       
#define MAX_STICK 1673      
#define MIN_DEADBAND 1019
#define MAX_DEADBAND 1029
#define MID_STICK 1024
#define DIVIDER 2           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

RawSerial uart(PD_1,PD_0,9600);
//Serial pc(USBTX,USBRX,115200);

XWheels::XWheels()
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

int XWheels::Init()
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

void XWheels::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2])
{
  // unsigned int can store 16 bit int 
  StoreByte[0] = (Data & 0xFF00) >> 8;                  //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF);                       //Low byte, most left of HEX
}

long XWheels::map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float XWheels::LowRPMCompensation_Right(float InRPM)
{   
    // Only rpm from 5 to 40 
    float ComRPM;
    float E7 = 0.0000001;
    float E5 = 0.00001;
    float xR = abs(InRPM);     // Right Wheel input rpm
    float sign = abs(InRPM)/InRPM;

    //float rightErr = (-9.76*E7*(xR*xR*xR*xR*xR) + 0.000117*(xR*xR*xR*xR) - 0.00519*(xR*xR*xR) + 0.103*(xR*xR) - 0.999*xR + 5.96)*2.0;
    float rightErr = (-4.15*E7*(xR*xR*xR*xR*xR) + 4.65*E5*(xR*xR*xR*xR) - 0.00188*(xR*xR*xR) + 0.034*(xR*xR) - 0.357*xR + 4.06)*1.8;

    printf("rightErr %f \n", rightErr);
    ComRPM = InRPM + (sign)*abs(rightErr);

    return ComRPM;

}

float XWheels::LowRPMCompensation_Left(float InRPM)
{
    // Only rpm from 5 to 40 
    float ComRPM;
    float E7 = 0.0000001;
    float E5 = 0.00001;
    float xL = abs(InRPM);     // Left Wheel input rpm
    float sign = abs(InRPM)/InRPM;

    float leftErr = (-4.15*E7*(xL*xL*xL*xL*xL) + 4.65*E5*(xL*xL*xL*xL) - 0.00188*(xL*xL*xL) + 0.034*(xL*xL) - 0.357*xL + 4.06)*1.85;
    
    printf("leftErr %f \n", leftErr);
    ComRPM = InRPM + (sign)*abs(leftErr);

    return ComRPM;
}

unsigned int XWheels::RPMToRaw(float rpm)
{
  int raw_int;
  unsigned int out_raw;


    // map rpm to raw value
    raw_int = (int)map(rpm, 0.0, 144.0, 0, 3200);   // for linear approximate, but low RPM is not collect
 
  
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

unsigned int XWheels::RPMToRaw2(float rpm)
{
  int raw_int;
  unsigned int out_raw;

    if (rpm >= 0.0 && rpm < 40.0)
    {
        // map rpm to raw value
        raw_int = (int)map(rpm, 0.0, 39.0, 70, 866);
    }
    else if(rpm < 0.0 && rpm > -40.0)
    {
        raw_int = (int)map(rpm, 0.0, -39.0, 32845, 33634); 
    }
    else if(rpm >= 40.0)
    {
        raw_int = (int)map(rpm, 40.0, 144.0, 888, 3200); 
    }
    else if(rpm <= -40.0)
    {
        raw_int = (int)map(rpm, -40.0, -144.0, 33656, 35968); 
        //out_raw = 32768 + abs(raw_int);
    }

    out_raw = raw_int;
   
    
  return out_raw;
}
void XWheels::waitUntilFourZero()
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

void XWheels::ESCHandShake()
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

void XWheels::zeroSpeed()
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

void XWheels::DriveWheels(float rpm1, float rpm2)
{  
    float Out_RPM_Right;
    float Out_RPM_Left;
    /*
    // Both rpm right and left are low
    if (abs(rpm1) >= 5.0 && abs(rpm1) <=40.0 && abs(rpm2) >= 5.0 && abs(rpm2) <=40.0)
    {
        Out_RPM_Right = LowRPMCompensation_Right(rpm1);
        Out_RPM_Left = LowRPMCompensation_Left(rpm2);

    }
    else if(abs(rpm1) >= 5.0 && abs(rpm1) <=40.0)
    {
        Out_RPM_Right = LowRPMCompensation_Right(rpm1);
        Out_RPM_Left = rpm2;
    }
    
    else if (abs(rpm2) >= 5.0 && abs(rpm2) <=40.0)
    {   
        Out_RPM_Right = rpm1;
        Out_RPM_Left = LowRPMCompensation_Left(rpm2);
    }
    else
    {
        Out_RPM_Right = rpm1;
        Out_RPM_Left = rpm2;
    }
    printf("Out_Right %f\n", Out_RPM_Right);
    printf("Out_Left %f\n", Out_RPM_Left);
    */
    
    //Out_RPM_Right = rpm1;
    //Out_RPM_Left = rpm2;
    RawInt1 = RPMToRaw2(rpm1);
    RawInt2 = RPMToRaw2(rpm2);
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

void XWheels::vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2])
{   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

    /////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND)
    {
        MotorRPM[0] = 0.0;
        MotorRPM[1] = 0.0;
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && (UD_ch > MAX_DEADBAND || UD_ch < MIN_DEADBAND))
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[1] = MotorRPM[0];

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND && (LR_ch >= MAX_DEADBAND || LR_ch <= MIN_DEADBAND))
    {
        MotorRPM[1] = (float)map(LR_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[0] = -MotorRPM[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = (float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, 100.0, 200.0);
        MotorRPM[0] = MotorRPM[1]*100.0/SCALE;
        //printf("SCALE %f\n",SCALE);
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, 100.0, 200.0);
        MotorRPM[1] = MotorRPM[0]*100.0/SCALE;
        //printf("SCALE %f\n",SCALE);
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, 100.0, 200.0);
        MotorRPM[1] = MotorRPM[0]*100.0/SCALE;
        //printf("SCALE %f\n",SCALE);
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = (float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, 100.0, 200.0);
        MotorRPM[0] = MotorRPM[1]*100.0/SCALE;
        //printf("SCALE %f\n",SCALE);
    }  
   
}