#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <LMotorController.h>
#include <Arduino.h>

class PID {
public:
    PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection);
    void Compute();
    void SetTunings(double Kp, double Ki, double Kd);
    void SetSampleTime(int NewSampleTime);
    void SetOutputLimits(double Min, double Max);
    void SetMode(int Mode);
    void SetControllerDirection(int Direction);

private:
    double *myInput, *myOutput, *mySetpoint;
    double kp, ki, kd;
    double ITerm, lastInput;
    unsigned long lastTime;
    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;
    int controllerDirection;
};

#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1

PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection) {
    myInput = Input;
    myOutput = Output;
    mySetpoint = Setpoint;

    inAuto = false;

    PID::SetOutputLimits(0, 255); // Default output limits
    SampleTime = 100;             // Default sample time is 100ms

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis() - SampleTime;
}

void PID::Compute() {
    if (!inAuto) return;

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);

    if (timeChange >= SampleTime) {
        double input = *myInput;
        double error = *mySetpoint - input;
        
        ITerm += (ki * error);
        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;

        double dInput = (input - lastInput);

        double output = kp * error + ITerm - kd * dInput;
        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;

        *myOutput = output;

        lastInput = input;
        lastTime = now;
    }
}

void PID::SetTunings(double Kp, double Ki, double Kd) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    double SampleTimeInSec = ((double)SampleTime) / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    if (controllerDirection == REVERSE) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
}

void PID::SetSampleTime(int NewSampleTime) {
    if (NewSampleTime > 0) {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

void PID::SetOutputLimits(double Min, double Max) {
    if (Min >= Max) return;
    outMin = Min;
    outMax = Max;

    if (inAuto) {
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;

        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;
    }
}

void PID::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        ITerm = *myOutput;
        lastInput = *myInput;
        if (ITerm > outMax) ITerm = outMax;
        else if (ITerm < outMin) ITerm = outMin;
    }
    inAuto = newAuto;
}

void PID::SetControllerDirection(int Direction) {
    if (inAuto && Direction != controllerDirection) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
    controllerDirection = Direction;
}

#define MIN_ABS_SPEED 30

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID setup
const double Kp = 29, Ki = 1300, Kd = 3;
double input, output, setpoint = 165;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int ENA = 32, IN1 = 25, IN2 = 33, IN3 = 27, IN4 = 26, ENB = 14;
double motorSpeedFactorLeft = 1, motorSpeedFactorRight = 1;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    Wire.setClock(400000L);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-0.21);
    mpu.setYGyroOffset(0.56);
    mpu.setZGyroOffset(0.17);
    mpu.setZAccelOffset(0.98);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(15, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180 / M_PI + 180;
    }
}
