#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <LMotorController.h>
#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>

#define LEDPIN 2

unsigned long previousMillis = 0;  // Stores the last time the setpoint was updated
const long interval = 1000;        // Interval for updating the setpoint (1 second)

const char* ssid = "KaftarKakolBeSarWiFi";         // WiFi SSID
const char* password = "88888888";          // WiFi Password
const char* mqtt_server = "broker.hivemq.com"; // MQTT broker
const int port = 1883;
unsigned long lastCommandTime = 0; // Tracks the last command time
const unsigned long commandTimeout = 500; // 0.5 seconds timeout

double lastTime = millis()-100;
int direction = 1;
int currentSpeed=0;
int inputCommand = 0;
double mySetpoint=165;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

WiFiClient espClient;
PubSubClient client(espClient);


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LEDPIN,1);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LEDPIN,0);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("rcontrol"); // Update this to your actual topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


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
  pinMode(LEDPIN,OUTPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();

  Wire.setClock(400000L);

  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Parse the received payload into an integer command
  int command = atoi((char*)payload);

  // Validate and set the command if it matches the expected values
  if (command >= 0 && command <= 4) {
    inputCommand = command;
    lastCommandTime = millis(); // Update the last command time
    Serial.print("Command Received: ");
    Serial.println(inputCommand);
  } else {
    Serial.println("Invalid command received!");
  }
}

void loop() {
    if (!dmpReady) return;
    Serial.println("1");
    if (!client.connected()) {
      reconnect();
      digitalWrite(LEDPIN,1);
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);
      Serial.println("2");
    }
    digitalWrite(LEDPIN,0);
    client.loop();
    Serial.println("3");

    // Check if the command timeout has been exceeded
    if (millis() - lastCommandTime > commandTimeout) {
      inputCommand = 0; // Reset to Stop
      Serial.println("4");
    }

    unsigned long currentMillis = millis();

    // Check if one second has passed
    //if (currentMillis - previousMillis >= interval) {
    //    previousMillis = currentMillis;  // Update the last time the setpoint was changed
    //    setpoint += 10;                  // Increment the setpoint by 10
    //}
    Serial.println("5");
    while (!mpuInterrupt && fifoCount < packetSize) {
        Serial.println("6");
        switch (inputCommand) {
          Serial.println("7");
          case 0: // Stop
             Serial.println("Stop");
             setpoint = 165;
             Serial.println("8");
          break;
          case 1: // Forward
            Serial.println("Move Forward");
            setpoint = 180;
            Serial.println("9");
          break;
        case 2: // Backward
            Serial.println("Move Backward");
            setpoint = 150;
            Serial.println("10");
        break;
        case 3: // Right
            Serial.println("Turn Right");
            setpoint = 165;
            Serial.println("11");
        break;
        case 4: // Left
            Serial.println("Turn Left");
            setpoint = 165;
            Serial.println("12");
        break;
      }
        Serial.println("13");
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }
    Serial.println("14");
    if (mpuInterrupt) {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        Serial.println("15");
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize){
           fifoCount = mpu.getFIFOCount();
          Serial.println("16");
          }

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180 / M_PI + 180;
    }
    }
}
