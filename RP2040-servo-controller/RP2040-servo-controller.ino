#include <Adafruit_ISM330DHCX.h>
#include <Kalman.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// Hydraulic valve pins (Servo motors)
#define VALVE_PIN 26

// NeoPixel setup
int Power = 11;
int PIN = 12;
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Timing and PID intervals
unsigned long prevMillisValve = 0;
const int valveInterval = 20;  // Interval for updating valves in milliseconds (50Hz)

// Kalman Filter Angles
double kalAngleX, kalAngleY;

// PID Setpoints and Variables
double Input, Output, Setpoint;

// Default PID tuning parameters
double Kp, Ki, Kd;

// Default servo parameters
int servo_center;
double lim, deadzone;

// PID object
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// IMU and Kalman filter setup
Adafruit_ISM330DHCX ism330dhcx;
Kalman kalmanX, kalmanY;

// Servo setup
Servo valve;

// default addr
#define IMU_ADDRESS 0x6A

// Declaration for IMU update timer
unsigned long timer = micros();

// Sensor reader class
class SensorReader {
public:
  SensorReader(Adafruit_ISM330DHCX& sensor) : _sensor(sensor) {}

  void readData() {
    sensors_event_t accel, gyro, temp;
    _sensor.getEvent(&accel, &gyro, &temp);
    accX = accel.acceleration.x;
    accY = accel.acceleration.y;
    accZ = accel.acceleration.z;
    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;
    tempRaw = temp.temperature;
  }

  double accX, accY, accZ;
  double gyroX, gyroY, gyroZ;
  int16_t tempRaw;

private:
  Adafruit_ISM330DHCX& _sensor;
};

SensorReader sensor(ism330dhcx);

void setup() {

  // NeoPixel setup
  pixels.begin();
  pinMode(Power, OUTPUT);
  digitalWrite(Power, HIGH);

  // Set RGB LED to red during setup
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();

  // Check IMU initialization first
  if (!ism330dhcx.begin_I2C(IMU_ADDRESS)) {
    Serial.println("Failed to initialize IMU!");

    while (1) {
        pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Turn on red light
        pixels.show();
        delay(500); // Wait for 500 milliseconds
        pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Turn off light
        pixels.show();
        delay(500); // Wait for 500 milliseconds
    }
  }


  Serial.begin(115200);

  // Wait for the initial settings to be sent over serial
  while (!Serial.available()) {
    delay(10);
  }


  // Set RGB LED to amber while waiting for the initial settings to be sent over serial
  pixels.setPixelColor(0, pixels.Color(255, 194, 0));
  pixels.show();

  // read settings from serial
  readSerial();

  // Now we can attach the valve and initialize the IMU with the received settings
  valve.attach(VALVE_PIN, 500, 2500);
  valve.write(servo_center);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(servo_center - lim, servo_center + lim);
  pid.SetSampleTime(valveInterval);

  updateIMUAngles(sensor, kalmanX, kalmanY, kalAngleX, kalAngleY, timer);

  // Set RGB LED to green when running
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
}

void loop() {
  if (Serial.available() > 0) {
    readSerial();
  }

  unsigned long currentMillis = millis();

  // valve control
  if (currentMillis - prevMillisValve >= valveInterval) {
    updateIMUAngles(sensor, kalmanX, kalmanY, kalAngleX, kalAngleY, timer);
    //Input = kalAngleX;
    Input = kalAngleY;
    pid.Compute();

    // Apply deadzone
    if (abs(Input - Setpoint) > deadzone) {
      int servoAngle = servo_center - (Output - servo_center);
      servoAngle = constrain(servoAngle, (servo_center - lim), (servo_center + lim));
      valve.write(servoAngle);
    } else {
      valve.write(servo_center);
    }

    prevMillisValve = currentMillis;
  }

  sendSerialData();
}

void updateIMUAngles(SensorReader& sensor, Kalman& kalmanX, Kalman& kalmanY, double& kalAngleX, double& kalAngleY, unsigned long& timer) {
  sensor.readData();
  double dt = (micros() - timer) / 1000000.0;
  timer = micros();

  double roll = atan2(sensor.accY, sensor.accZ) * RAD_TO_DEG;
  double pitch = atan2(-sensor.accX, sensor.accZ) * RAD_TO_DEG;

  double gyroXrate = sensor.gyroX;
  double gyroYrate = sensor.gyroY;

  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
}

void readSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    char inputCopy[input.length() + 1];
    input.toCharArray(inputCopy, sizeof(inputCopy));

    char* token = strtok(inputCopy, ",");
    double incoming_values[9];

    // Extract values into incoming_values array
    int i = 0;
    while (token != NULL && i < 9) {
      incoming_values[i++] = atof(token);
      token = strtok(NULL, ",");
    }

    // Check if PID parameters have changed
    bool pidParamsChanged = false;
    if (incoming_values[3] != Kp || incoming_values[4] != Ki || incoming_values[5] != Kd || incoming_values[6] != servo_center || incoming_values[7] != lim) {
      pidParamsChanged = true;
    }

    

    // Map values to the correct variables
    Input = incoming_values[0];
    Output = incoming_values[1];
    Setpoint = incoming_values[2];
    Kp = incoming_values[3];
    Ki = incoming_values[4];
    Kd = incoming_values[5];
    servo_center = static_cast<int>(incoming_values[6]);  // servo.write wants integers!
    lim = incoming_values[7];
    deadzone = incoming_values[8];

    // Update PID parameters only if they've changed
    if (pidParamsChanged) {
      pid.SetTunings(Kp, Ki, Kd);
      pid.SetOutputLimits(servo_center - lim, servo_center + lim);
    }
  }
}

void sendSerialData() {
  String data = String(Input) + "," + String(Output) + "," + String(Setpoint) + "," + 
                String(Kp) + "," + String(Ki) + "," + String(Kd) + "," + 
                String(servo_center) + "," + String(lim) + "," + String(deadzone) + "\n";
  Serial.print(data);
}