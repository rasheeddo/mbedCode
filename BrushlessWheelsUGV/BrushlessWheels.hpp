#ifndef __BRUSHLESS_WHEELS_HPP
#define __BRUSHLESS_WHEELS_HPP

class BrushlessWheels
{
    public:
    BrushlessWheels();
    //class constructor to initialize baudrate setting 

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Data Converting function /////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]);
    // Int16ToByteData: split an unsigned int into 2 bytes data
    // unsigned int stores 16 bits as an integer
    // Pass by reference using array StoreByte[2], this is output of the function

    unsigned int RPMToRaw(float rpm);
    // RPMToRaw: convert rotation per minute value to raw value
    // input is a float number between -MaxRPM to MaxRPM, minus sign means reverse rotation
    // output is a value from 0 to 3200 for forward rotation and 32769 to 35968 for reverse rotation

    long map(long x, long in_min, long in_max, long out_min, long out_max);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Hand Shake with ESC //////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    int Init();
    // Init: initialize by do waitUntilFourByte, ESCHandShake and zeroSpeed
    // MUST DO inside the setup() loop on your sketch

    void waitUntilFourZero();
    // waitUntilFourZero: when start, ESC will send four bytes of zero then wait for our controller to response
    // if our control received four bytes of zero, then it will allow next command to run after

    void ESCHandShake();
    // ESCHandShake: from hack, we need to send 17bytes of "Hand-Shake" style 20times with specific delay

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////// Drive function ////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void zeroSpeed();
    // zeroSpeed: set motor to zero rpm, this will be used when start after hand shake

    void DriveWheels(float rpm1, float rpm2);
    // DriveWheels: drive both wheels with desired rpm, + for forward - for reverse rotations
        
        

    private:
        int i;
        bool ReadOK;
        char Reply[6];     // initial value
        bool startTick;
        int count;

        float MaxRPM;
        unsigned int RawInt1;
        unsigned int RawInt2;
        unsigned char Motor1SpeedByte[2];
        unsigned char Motor2SpeedByte[2];

        unsigned char Header1;
        unsigned char Header2;


};


#endif