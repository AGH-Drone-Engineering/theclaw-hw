#include <Arduino.h>
#include <MotorDC.h>

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#define PWM_WRITE(pin, value) analogWrite(pin, value)

MotorDC::MotorDC(int pinA, int pinB)
    : m_pinA(pinA), m_pinB(pinB)
{
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
}

void MotorDC::drive(int speed)
{
    speed = CLAMP(speed, -255, 255);
    if (speed >= 0)
    {
        PWM_WRITE(m_pinA, speed);
        digitalWrite(m_pinB, LOW);
    }
    else if (speed < 0)
    {
        PWM_WRITE(m_pinA, 255 + speed);
        digitalWrite(m_pinB, HIGH);
    }
}
