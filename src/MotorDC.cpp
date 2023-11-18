#include <Arduino.h>
#include <MotorDC.h>

MotorDC::MotorDC(int pinA, int pinB)
    : m_pinA(pinA), m_pinB(pinB)
{}

void MotorDC::init()
{
    pinMode(m_pinA, OUTPUT);
    pinMode(m_pinB, OUTPUT);
}

void MotorDC::drive(int speed)
{
    speed = constrain(speed, -255, 255);
    if (speed >= 0)
    {
        analogWrite(m_pinA, speed);
        analogWrite(m_pinB, 0);
    }
    else if (speed < 0)
    {
        analogWrite(m_pinA, 0);
        analogWrite(m_pinB, -speed);
    }
}
