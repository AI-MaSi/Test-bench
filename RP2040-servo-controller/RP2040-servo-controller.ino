#include <Adafruit_ISM330DHCX.h>
#include <Kalman.h>
#include <PID_v1.h>
#include <Servo.h>

// Hydraulic valve pins (Servo motors)
#define VALVE1_PIN 26
#define VALVE2_PIN 27

// Timing and PID intervals
unsigned long prevMillisValve1 = 0;
unsigned long prevMillisValve2 = 0;  // Add timing for the second valve
const int valveInterval = 20;  // Interval for updating valves in milliseconds (50Hz)

// Kalman Filter Angles
double kalAngleX, kalAngleY;  // Angles calculated by the Kalman filter
double kalAngleX2, kalAngleY2;  // Add angles for the second IMU

// PID Setpoints and Variables
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;  // Add PID variables for the second valve

// PID tuning parameters
float Kp1 = 2.0, Ki1 = 5.0, Kd1 = 1.0;  // PID constants for valve 1
float Kp2 = 2.0, Ki2 = 5.0, Kd2 = 1.0;  // PID constants for valve 2

// Servo parameters
int servo_center = 90;  // Center position of the servo
int lim = 30;  // Limit of the servo

// PID objects
PID pid1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID pid2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);  // Add PID object for the second valve

// IMU and Kalman filter setup
Adafruit_ISM330DHCX ism330dhcx;
Adafruit_ISM330DHCX ism330dhcx2;  // Add a second IMU
Kalman kalmanX, kalmanY;
Kalman kalmanX2, kalmanY2;  // Add Kalman filter objects for the second IMU

// Servo setup
Servo valve1;
Servo valve2;  // Add a second servo

// Declaration for IMU update timer
unsigned long timer = micros();
unsigned long timer2 = micros();  // Add a timer for the second IMU

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

SensorReader sensor1(ism330dhcx);  // Instance of SensorReader
SensorReader sensor2(ism330dhcx2);  // Add a second instance for the second IMU

void setup() {
  Serial.begin(115200);
  valve1.attach(VALVE1_PIN, 500, 2500); // Attach the servo on the hydraulic valve pin. min max micros
  valve1.write(servo_center);  // Set servo to middle
  valve2.attach(VALVE2_PIN, 500, 2500); // Attach the second servo
  valve2.write(servo_center);  // Set second servo to middle

  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to initialize IMU1!");
    while (1) delay(10);
  }

  if (!ism330dhcx2.begin_I2C()) {
    Serial.println("Failed to initialize IMU2!");
    while (1) delay(10);
  }

  // Set the PID controllers to automatic mode to start processing input right away
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);

  // Set output limits to match the servo's range
  pid1.SetOutputLimits((servo_center-lim), (servo_center+lim));
  pid2.SetOutputLimits((servo_center-lim), (servo_center+lim));

  // Calculate PID every 20 milliseconds
  pid1.SetSampleTime(valveInterval);
  pid2.SetSampleTime(valveInterval);

  // Initialize IMU angle readings
  updateIMUAngles();
  updateIMUAngles2();

}

void loop() {
  if (Serial.available() > 0) {
    readSerialSetpoints();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - prevMillisValve1 >= valveInterval) {
    updateIMUAngles();
    Input1 = kalAngleY; // or kalAngleX
    pid1.Compute();

    int servo1Angle = servo_center - (Output1 - servo_center); // Invert the PID output
    servo1Angle = constrain(servo1Angle, (servo_center-lim), (servo_center+lim)); // Ensure the servo angle is within 60 to 120 degrees
    valve1.write(servo1Angle); // Write the constrained and inverted angle to the servo

    // Print the current PID constants, Input, Setpoint, and Output
    Serial.print("Input1 (KalY): "); Serial.print(Input1);
    Serial.print("\tSetpoint1 angle: "); Serial.print(Setpoint1);
    Serial.print("\tOutput1 (Servo1 Angle): "); Serial.print(servo1Angle);
    Serial.print("\tKp1: "); Serial.print(Kp1);
    Serial.print("\tKi1: "); Serial.print(Ki1);
    Serial.print("\tKd1: "); Serial.println(Kd1);

    prevMillisValve1 = currentMillis;
  }

  if (currentMillis - prevMillisValve2 >= valveInterval) {
    updateIMUAngles2();
    Input2 = kalAngleY2; // or kalAngleX2
    pid2.Compute();

    int servo2Angle = servo_center - (Output2 - servo_center); // Invert the PID output
    servo2Angle = constrain(servo2Angle, (servo_center-lim), (servo_center+lim)); // Ensure the servo angle is within 60 to 120 degrees
    valve2.write(servo2Angle);

    // Print the current PID constants, Input, Setpoint, and Output for the second valve
    Serial.print("Input2 (KalY2): "); Serial.print(Input2);
    Serial.print("\tSetpoint2 angle: "); Serial.print(Setpoint2);
    Serial.print("\tOutput2 (Servo2 Angle): "); Serial.print(servo2Angle);
    Serial.print("\tKp2: "); Serial.print(Kp2);
    Serial.print("\tKi2: "); Serial.print(Ki2);
    Serial.print("\tKd2: "); Serial.println(Kd2);

    prevMillisValve2 = currentMillis;
  }
}

void updateIMUAngles() {
  sensor1.readData();
  double dt = (micros() - timer) / 1000000.0;
  timer = micros();

  double roll = atan2(sensor1.accY, sensor1.accZ) * RAD_TO_DEG;
  double pitch = atan2(-sensor1.accX, sensor1.accZ) * RAD_TO_DEG;

  double gyroXrate = sensor1.gyroX;
  double gyroYrate = sensor1.gyroY;

  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
}

void updateIMUAngles2() {
  sensor2.readData();
  double dt = (micros() - timer2) / 1000000.0;
  timer2 = micros();

  double roll = atan2(sensor2.accY, sensor2.accZ) * RAD_TO_DEG;
  double pitch = atan2(-sensor2.accX, sensor2.accZ) * RAD_TO_DEG;

  double gyroXrate = sensor2.gyroX;
  double gyroYrate = sensor2.gyroY;

  kalAngleX2 = kalmanX2.getAngle(roll, gyroXrate, dt);
  kalAngleY2 = kalmanY2.getAngle(pitch, gyroYrate, dt);
}

void readSerialSetpoints() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // haha awful code
    if (input.startsWith("setpoint1=")) {
      Setpoint1 = input.substring(10).toDouble();
      Serial.print("Updated Setpoint1 to: ");
      Serial.println(Setpoint1);
    } else if (input.startsWith("P1=")) {
      Kp1 = input.substring(3).toFloat();
      Serial.print("Updated Kp1 to: ");
      Serial.println(Kp1);
    } else if (input.startsWith("I1=")) {
      Ki1 = input.substring(3).toFloat();
      Serial.print("Updated Ki1 to: ");
      Serial.println(Ki1);
    } else if (input.startsWith("D1=")) {
      Kd1 = input.substring(3).toFloat();
      Serial.print("Updated Kd1 to: ");
      Serial.println(Kd1);
    } else if (input.startsWith("setpoint2=")) {
      Setpoint2 = input.substring(10).toDouble();
      Serial.print("Updated Setpoint2 to: ");
      Serial.println(Setpoint2);
    } else if (input.startsWith("P2=")) {
      Kp2 = input.substring(3).toFloat();
      Serial.print("Updated Kp2 to: ");
      Serial.println(Kp2);
    } else if (input.startsWith("I2=")) {
      Ki2 = input.substring(3).toFloat();
      Serial.print("Updated Ki2 to: ");
      Serial.println(Ki2);
    } else if (input.startsWith("D2=")) {
      Kd2 = input.substring(3).toFloat();
      Serial.print("Updated Kd2 to: ");
      Serial.println(Kd2);
    } else if (input.startsWith("servo_lim=")) {
        lim = input.substring(10).toInt();
        Serial.print("Updated servo limit to: ");
        Serial.println(lim);
    }
    pid1.SetTunings(Kp1, Ki1, Kd1);  // Apply the new PID tunings if any were updated
    pid2.SetTunings(Kp2, Ki2, Kd2);  // Apply the new PID tunings for the second valve if any were updated

    // Update servo limits
    pid1.SetOutputLimits((servo_center-lim), (servo_center+lim));
    pid2.SetOutputLimits((servo_center-lim), (servo_center+lim));
  }
}
