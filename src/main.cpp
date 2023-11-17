#ifdef APP_USE_USBCON
#define USE_USBCON
#endif

#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <Encoder.h>
#include <ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Range.h>

#include <MotorDC.h>

#define MOTOR_A1 5
#define MOTOR_A2 6

#define ENCODER_A1 2
#define ENCODER_A2 4

#define MOTOR_B1 9
#define MOTOR_B2 10

#define ENCODER_B1 3
#define ENCODER_B2 7

#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO 12

#define GRIPPER_PIN 11

static ros::NodeHandle nh;

class EncoderManager
{
public:
    EncoderManager(int pinA, int pinB, const char *posTopic, const char *speedTopic)
    : encoder(pinA, pinB)
    , pos(0)
    , lastRead(0)
    , posPub(posTopic, &posMsg)
    , speedPub(speedTopic, &speedMsg)
    {}

    void init()
    {
        nh.advertise(posPub);
        nh.advertise(speedPub);
    }

    void process()
    {
        unsigned long now = millis();
        int32_t delta = encoder.readAndReset();
        pos += delta;

        posMsg.data = pos;
        posPub.publish(&posMsg);

        speedMsg.data = delta * 1000 / (now - lastRead);
        speedPub.publish(&speedMsg);

        lastRead = now;
    }

private:
    Encoder encoder;
    int32_t pos;
    unsigned long lastRead;
    std_msgs::Int32 posMsg;
    ros::Publisher posPub;
    std_msgs::Int32 speedMsg;
    ros::Publisher speedPub;
};

// Motor A

static MotorDC motorA(MOTOR_A1, MOTOR_A2);

static void motorACmdCallback(const std_msgs::Int16 &command)
{
    motorA.drive(command.data);
}

static ros::Subscriber<std_msgs::Int16> motorACmdSub("motor_a/command", motorACmdCallback);

// Encoder A

static EncoderManager encoderA(ENCODER_A1, ENCODER_A2, "motor_a/position", "motor_a/speed");

// Motor B

static MotorDC motorB(MOTOR_B1, MOTOR_B2);

static void motorBCmdCallback(const std_msgs::Int16 &command)
{
    motorB.drive(command.data);
}

static ros::Subscriber<std_msgs::Int16> motorBCmdSub("motor_b/command", motorBCmdCallback);

// Encoder B

static EncoderManager encoderB(ENCODER_B1, ENCODER_B2, "motor_b/position", "motor_b/speed");

// Ultrasonic

static Ultrasonic ultrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);

static sensor_msgs::Range ultrasonicMsg;
static ros::Publisher ultrasonicPub("ultrasonic", &ultrasonicMsg);

static void processUltrasonic()
{
    ultrasonicMsg.header.stamp = nh.now();
    ultrasonicMsg.header.frame_id = "ultrasonic";
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

    encoderA.init();
    encoderB.init();

    gripper.attach(GRIPPER_PIN);
    gripper.write(0);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    processUltrasonic();
    encoderA.process();
    encoderB.process();
    nh.spinOnce();
}
