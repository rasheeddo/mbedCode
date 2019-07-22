#include "mbed.h"
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

// main() runs in its own thread in the OS
int main()
{
    while (true) {
        led1 = 1;
        led2 = 1;
        led3 = 1;
        wait_ms(500);
        led1 = 0;
        led2 = 0;
        led3 = 0;
        wait_ms(500);

    }
}
