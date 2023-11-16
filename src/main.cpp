#define USE_USBCON

#include <Arduino.h>
#include <SoftPWM.h>
#include <ros.h>

#include <std_msgs/Int16.h>

#include <MotorDC.h>

#define MOTOR_A1 11 // D11
#define MOTOR_A2 10 // D10

#define MOTOR_B1 9  // D9
#define MOTOR_B2 3  // SCL

void motorSpeedCallback(MotorDC *motor, const std_msgs::Int16 &speed);

MotorDC motorA(MOTOR_A1, MOTOR_A2);
MotorDC motorB(MOTOR_B1, MOTOR_B2);

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int16> motorASpeedSub("motor_a/command",
    [](const std_msgs::Int16 &speed) {
        motorSpeedCallback(&motorA, speed);
    });

ros::Subscriber<std_msgs::Int16> motorBSpeedSub("motor_b/command",
    [](const std_msgs::Int16 &speed) {
        motorSpeedCallback(&motorB, speed);
    });

void setup()
{
    pinMode(13, OUTPUT);

    SoftPWMBegin();

    nh.initNode();
    nh.subscribe(motorASpeedSub);
    nh.subscribe(motorBSpeedSub);
}

void loop()
{
    nh.spinOnce();
}

void motorSpeedCallback(MotorDC *motor, const std_msgs::Int16 &speed)
{
    motor->drive(speed.data);
}
