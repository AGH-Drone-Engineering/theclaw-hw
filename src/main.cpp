#ifdef APP_USE_USBCON
#define USE_USBCON
#endif

#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>

#include <MotorDC.h>

#define MOTOR_A1 11 // D11
#define MOTOR_A2 10 // D10

#define ENCODER_A1 2 // ?
#define ENCODER_A2 3 // ?

#define MOTOR_B1 9  // D9
#define MOTOR_B2 3  // SCL

#define ENCODER_B1 7 // ?
#define ENCODER_B2 8 // ?

#define ULTRASONIC_TRIG 4 // ?
#define ULTRASONIC_ECHO 5 // ?

#define GRIPPER_PIN 6 // ?

static ros::NodeHandle nh;

// Motor A

static MotorDC motorA(MOTOR_A1, MOTOR_A2);

static void motorACmdCallback(const std_msgs::Int8 &command)
{
    motorA.drive(command.data);
}

static ros::Subscriber<std_msgs::Int8> motorACmdSub("motor_a/command", motorACmdCallback);

// Motor B

static MotorDC motorB(MOTOR_B1, MOTOR_B2);

static void motorBCmdCallback(const std_msgs::Int8 &command)
{
    motorB.drive(command.data);
}

static ros::Subscriber<std_msgs::Int8> motorBCmdSub("motor_b/command", motorBCmdCallback);

// Ultrasonic

static Ultrasonic ultrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);

static sensor_msgs::Range ultrasonicMsg;
static ros::Publisher ultrasonicPub("ultrasonic", &ultrasonicMsg);

static void processUltrasonic()
{
    memset(&ultrasonicMsg, 0, sizeof(ultrasonicMsg));
    ultrasonicMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    ultrasonicMsg.field_of_view = 18.f / 180.f * M_PI;
    ultrasonicMsg.min_range = 0.f;
    ultrasonicMsg.max_range = 1.f;
    ultrasonicMsg.range = ultrasonic.read() * 0.01f;
    ultrasonicPub.publish(&ultrasonicMsg);
}

// Gripper

static Servo gripper;

static void gripperCmdCallback(const std_msgs::UInt8 &command)
{
    int angle = constrain(command.data, 0, 180);
    gripper.write(angle);
}

static ros::Subscriber<std_msgs::UInt8> gripperCmdSub("gripper/command", gripperCmdCallback);

// Main

void setup()
{
    nh.initNode();
    nh.subscribe(motorACmdSub);
    nh.subscribe(motorBCmdSub);
    nh.subscribe(gripperCmdSub);
    nh.advertise(ultrasonicPub);

    gripper.attach(GRIPPER_PIN);
}

void loop()
{
    processUltrasonic();
    nh.spinOnce();
}
