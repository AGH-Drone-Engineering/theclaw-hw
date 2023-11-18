#ifndef _MOTORDC_H
#define _MOTORDC_H


class MotorDC {
public:
    MotorDC(int pinA, int pinB);

    void init();

    void drive(int speed);

private:
    int m_pinA;
    int m_pinB;
};


#endif // _MOTORDC_H
