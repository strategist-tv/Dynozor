#include <SCServo.h>

// ESP32 UART pins
#define S_RXD 18
#define S_TXD 19

// Servo settings
#define SERVO_ID 3
#define BAUD_RATE 8000

SCSCL sc;

void setup()
{
    // Start ESP32 UART1
    Serial1.begin(BAUD_RATE, SERIAL_8N1, S_RXD, S_TXD);

    // Tell SCServo library to use Serial1
    sc.pSerial = &Serial1;

    delay(1000);
}

void loop()
{
    // Move to 0 degrees
    sc.WritePosEx(SERVO_ID, 0, 1000, 50);

    // Give it time to reach the position
    delay(2000);

    // Move to 180 degrees
    sc.WritePosEx(SERVO_ID, 2048, 1000, 50);

    // Give it time to reach the position
    delay(2000);
}
