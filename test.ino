#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LMotorController.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>

// WebSocket Definitions
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

// Motor Controller and MPU Definitions
LMotorController motorController(5, 4, 18, 19, 100); // Example GPIO pins
MPU6050 mpu;

// PID Definitions
double input, output, setpoint;
double originalSetpoint = 173; // Adjust as needed
double movingAngleOffset = 0;
double Kp = 60, Ki = 5, Kd = 15;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// MPU Variables
bool dmpReady = false;
uint8_t mpuIntStatus, devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Command Variables
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
int inputCommand = STOP;

// HTML Interface
const char* htmlHomePage PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<style>
body { text-align: center; font-family: Arial, sans-serif; }
table { margin: auto; }
td { padding: 20px; background: #ccc; border: 1px solid #999; cursor: pointer; }
td:hover { background: #aaa; }
</style>
</head>
<body>
<h1>Mini Segway Control</h1>
<table>
  <tr>
    <td></td>
    <td onclick='sendButtonInput("MoveCar,1")'>Forward</td>
    <td></td>
  </tr>
  <tr>
    <td onclick='sendButtonInput("MoveCar,3")'>Left</td>
    <td onclick='sendButtonInput("MoveCar,0")'>Stop</td>
    <td onclick='sendButtonInput("MoveCar,4")'>Right</td>
  </tr>
  <tr>
    <td></td>
    <td onclick='sendButtonInput("MoveCar,2")'>Backward</td>
    <td></td>
  </tr>
</table>
<script>
function sendButtonInput(command) {
    var ws = new WebSocket('ws://' + location.hostname + '/CarInput');
    ws.onopen = function() { ws.send(command); };
    ws.onclose = function() { ws = null; };
}
</script>
</body>
</html>
)HTML";

// WebSocket Event Handler
void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                              AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->opcode == WS_TEXT) {
            String msg = String((char *)data);
            if (msg.startsWith("MoveCar,")) {
                inputCommand = msg.substring(8).toInt();
            }
        }
    }
}

// Handle Commands
void handleMovement() {
    switch (inputCommand) {
        case FORWARD:
            movingAngleOffset = -5; // Slight forward tilt
            break;
        case BACKWARD:
            movingAngleOffset = 5; // Slight backward tilt
            break;
        case LEFT:
            motorController.move(80, -80); // Left motor slow, right motor fast
            break;
        case RIGHT:
            motorController.move(-80, 80); // Left motor fast, right motor slow
            break;
        case STOP:
        default:
            movingAngleOffset = 0;
            motorController.move(0, 0); // Stop
            break;
    }
}

// Setup Function
void setup() {
    Serial.begin(115200);

    // Initialize MPU
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.println("DMP Initialization failed!");
        while (1);
    }

    // Initialize PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    // Setup Web Server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", htmlHomePage);
    });
    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);
    server.begin();
}

// Main Loop
void loop() {
    if (!dmpReady) return;

    if (mpu.getFIFOCount() < packetSize) return;

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180 / M_PI + 180; // Convert to degrees
    setpoint = originalSetpoint + movingAngleOffset;
    pid.Compute();

    motorController.move(output, output); // Adjust motors
    handleMovement();
}
