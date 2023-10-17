#include "SerialTransfer.h"
#include <NewPing.h>
#include <QMC5883LCompass.h>
#include <Wire.h>

// ===================================================================

// ------------------ Serial ------------------
SerialTransfer myTransfer;

struct __attribute__((packed)) CONTROLLER {
  long pitch;
  long yaw;
  long roll;
  long throttle;
  long aux1;
  long aux2;
  long aux3;
  long aux4;
} commandStruct;



struct __attribute__((packed)) SENSORS {
  float sonar1;
  float sonar2;
  float sonar3;
  long heading;
  long ax;
} sensorStruct;

// ------------------ Sonar ------------------
#define SONAR1_TRIG  12  // D12 Arduino pin tied to trigger pin on the ultrasonic sensor.
#define SONAR1_ECHO  11  // D11 Arduino pin tied to echo pin on the ultrasonic sensor.
#define SONAR2_TRIG  8  // D8 Arduino pin tied to trigger pin on the ultrasonic sensor.
#define SONAR2_ECHO  7  // D7 Arduino pin tied to echo pin on the ultrasonic sensor.
#define SONAR3_TRIG  6  // D6 Arduino pin tied to trigger pin on the ultrasonic sensor.
#define SONAR3_ECHO  5  // D5 Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(SONAR1_TRIG, SONAR1_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(SONAR2_TRIG, SONAR2_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar3(SONAR3_TRIG, SONAR3_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int uS;

// ------------------ IMU ------------------
//Adafruit_MPU6050 mpu;

// ------------------ MAG ------------------
/* Assign a unique ID to this sensor at the same time */
QMC5883LCompass compass;

// ------------------ L298N ------------------

#define enA 9  // D9
#define in1 14 // A0
#define in2 15 // A1
#define enB 10 // D10
#define in3 16 // A2
#define in4 17 // A3





int motorSpeedA = 0;
int motorSpeedB = 0;

// ===================================================================

void setup()
{
  // ------------------ Serial ------------------
  Serial.begin(115200);
  myTransfer.begin(Serial);

  // ------------------ IMU ------------------
  // Try to initialize!
  //  if (!mpu.begin()) {
  //    Serial.println("Failed to find MPU6050 chip");
  //    while (1) {
  //      delay(10);
  //    }
  //  }
  //
  //  // set accelerometer range to +-8G
  //  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //
  //  // set gyro range to +- 500 deg/s
  //  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //
  //  // set filter bandwidth to 21 Hz
  //  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ------------------ MAG ------------------
  compass.init();
  compass.setCalibration(-1700, 840, 0, 3056, -2957, 0);


  // ------------------ L298N ------------------
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

// ===================================================================

void loop()
{
  if(myTransfer.available())
  {

    // ------------------ Receive Serial ------------------
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(commandStruct, recSize);
    


    // ------------------ L298N Bridge ------------------

    // Y-axis used for forward and backward control
    if (commandStruct.throttle < 1500) {
      // Set Motor A backward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Set Motor B backward
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      
      // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
      motorSpeedA = map(commandStruct.throttle, 1500, 0, 0, 255);
      motorSpeedB = map(commandStruct.throttle, 1500, 0, 0, 255);
    }
    
    else if (commandStruct.throttle > 1500) {
      // Set Motor A forward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      // Set Motor B forward
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
      motorSpeedA = map(commandStruct.throttle, 1500, 2000, 0, 255);
      motorSpeedB = map(commandStruct.throttle, 1500, 2000, 0, 255);
    }
    // If joystick stays in middle the motors are not moving
    else {
      motorSpeedA = 0;
      motorSpeedB = 0;
    }
  
    // X-axis used for left and right control
    if (commandStruct.yaw < 1500) {
      // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
      int xMapped = map(commandStruct.yaw, 1500, 0, 0, 255);
      // Move to left - decrease left motor speed, increase right motor speed
      motorSpeedA = motorSpeedA - xMapped;
      motorSpeedB = motorSpeedB + xMapped;
      // Confine the range from 0 to 255
      if (motorSpeedA < 0) {
        motorSpeedA = 0;
      }
      if (motorSpeedB > 255) {
        motorSpeedB = 255;
      }
      
    }
    if (commandStruct.yaw > 1500) {
      // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
      int xMapped = map(commandStruct.yaw, 1500, 2000, 0, 255);
      // Move right - decrease right motor speed, increase left motor speed
      motorSpeedA = motorSpeedA + xMapped;
      motorSpeedB = motorSpeedB - xMapped;
      // Confine the range from 0 to 255
      if (motorSpeedA > 255) {
        motorSpeedA = 255;
      }
      if (motorSpeedB < 0) {
        motorSpeedB = 0;
      }
    }

    // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
    if (motorSpeedA < 70) {
      motorSpeedA = 0;
    }
    if (motorSpeedB < 70) {
      motorSpeedB = 0;
    }

    analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
    analogWrite(enB, motorSpeedB); // Send PWM signal to motor B


    // ------------------ IMU and Compass ------------------

    long heading;
    // Read compass values
    compass.read();

    // Return Azimuth reading
    heading = compass.getAzimuth();

    sensorStruct.heading = float(heading);
    sensorStruct.ax = float(motorSpeedB);
    
    // ------------------ Sonar ------------------
  
    uS = sonar1.ping(); // Send ping, get ping time in microseconds (uS).
    sensorStruct.sonar1 = float(sonar1.convert_cm(uS));
  
    uS = sonar2.ping(); // Send ping, get ping time in microseconds (uS).
    sensorStruct.sonar2 = float(sonar2.convert_cm(uS));
  
    uS = sonar3.ping(); // Send ping, get ping time in microseconds (uS).
    sensorStruct.sonar3 = float(sonar3.convert_cm(uS));

    // ------------------ Send Serial ------------------
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(sensorStruct, sendSize);
    myTransfer.sendData(sendSize);

    
  }
}