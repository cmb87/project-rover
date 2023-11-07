#include "SerialTransfer.h"
#include <QMC5883LCompass.h>
//#include <Wire.h>

// ===================================================================
// https://www.youtube.com/watch?v=bsB5shP3vls

// ------------------ Serial ------------------
//SerialTransfer myTransfer;

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

// ------------------ MAG ------------------
/* Assign a unique ID to this sensor at the same time */
QMC5883LCompass compass;

// ------------------ L298N ------------------
// PWM Pins Nano: 3 5 6 9 10 11
#define enA 10  // D10 orange
#define in1 8 // D8 gelb
#define in2 9 // D9 grün
#define in3 7 // D17 lila
#define in4 12 // D12 weiß
#define enB 11 // D11 blau

// ------------------ Variable for Encoders ------------------
#define motorEncoderRS1 2  // D2 - yellow  : MUst be D2!!!
#define motorEncoderRS2 4  // D4 - green

#define motorEncoderLS1 3  // D6 - yellow  : Must be D3!!!
#define motorEncoderLS2 5  // D5 - green

// Volatile: Interrupt Service Routines (ISRs): When you're working with microcontrollers, ISRs are often used to handle 
// asynchronous events, such as button presses or timer events. Variables that are shared between the main program 
// and an ISR should be declared as volatile to ensure that the compiler doesn't optimize them away or cache their values in registers.

volatile int encoderRightCount = 0;
volatile int encoderLeftCount = 0;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// ------------------ Speed ------------------
// Motor encoder output pulses per 360 degree revolution (measured manually)
// 25GA370 130rpm https://mikroelectron.com/Product/25GA-370-12V-130RPM-REDUCER-GEAR-MOTOR-WITH-ENCODER/
// ticksPerRev 11 
#define ENC_COUNT_REV 495

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 // One-second interval for measurements
int interval = 300;

// Variable for RPM measuerment
float rpmR = 0;
float rpmL = 0;
float currentRpmR = 0;
float currentRpmL = 0;
const float alpha = 0.2 ; // EWMA smoothing constant;

// ------------------ Variable for PID Control ------------------
// https://tlk-energy.de/blog/pid-regler-einstellen
float kp = 10.0;
float kd = 1.0;
float ki = 0.4;

long previousTimeR = 0;
float ePreviousR = 0;
float eIntegralR = 0;

long previousTimeL = 0;
float ePreviousL = 0;
float eIntegralL = 0;

float ur = 0.0;
float ul = 0.0;

// ===================================================================

void handleEncoderR() {
    if (digitalRead(motorEncoderRS1)>digitalRead(motorEncoderRS2)) {
      if (encoderRightCount==encoder_maximum) {
        encoderRightCount = encoder_minimum;
      } else {
        encoderRightCount++;
      }
    } else {
      if (encoderRightCount==encoder_minimum) {
        encoderRightCount = encoder_maximum;
      } else {
        encoderRightCount--;
      }
    }
}

void handleEncoderL() {
    if (digitalRead(motorEncoderLS1)>digitalRead(motorEncoderLS2)) {
      if (encoderLeftCount==encoder_minimum) {
        encoderLeftCount = encoder_maximum;
      } else {
        encoderLeftCount--;
      }
    } else {
      if (encoderLeftCount==encoder_maximum) {
        encoderLeftCount = encoder_minimum;
      } else {
        encoderLeftCount++;
      }
    }
}

// ===================================================================

void setup()
{
  // ------------------ Serial ------------------
  Wire.begin();

  Serial.begin(115200);
  //myTransfer.begin(Serial);
  Serial.println("Beginning");
  // ------------------ MAG ------------------


  //compass.init();
  //compass.setCalibration(-1700, 840, 0, 3056, -2957, 0);
  Serial.println("Compass initialized");
  

  // ------------------ L298N ------------------
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);


  // ------------------ Encoders ------------------  
  pinMode(motorEncoderRS1, INPUT_PULLUP);
  pinMode(motorEncoderRS2, INPUT);
  pinMode(motorEncoderLS1, INPUT_PULLUP);
  pinMode(motorEncoderLS2, INPUT);

  // Interrupt for encoders => only triggered by rising signal!
  // Note on Arduino Nano interrupts are only on D2 and D3 see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  attachInterrupt(digitalPinToInterrupt(motorEncoderRS1), handleEncoderR, RISING);  
  attachInterrupt(digitalPinToInterrupt(motorEncoderLS1), handleEncoderL, RISING);  

  delay(3000);

}

// ===================================================================
float pidController(int actual, int target, float kp, float kd, float ki, float &eIntegral, float &ePrevious, long &previousTime) {

    // Measure the time elapsed since the last iteration
    long currentTime = micros();
    float deltaT = ((float)(currentTime-previousTime))/1.0e6;

    // Compute error, derivative and integral
    int e = actual-target;
    float eDerivative = (e-ePrevious)/deltaT;
    eIntegral = eIntegral + e*deltaT;

    // Compute PID control signal
    float u = (kp*e) + (kd*eDerivative) + (ki*eIntegral);

    // Update variables for next iteration
    previousTime = currentTime;
    ePrevious = e;

    return u;
}




// ===================================================================
void moveMotor(int dirPin1, int dirPin2, int enaPin, float u, int speedMin=70, int speedThres=10) {
  float speed = fabs(u); 
  if (speed > 255 ) {
    speed = 255;
  }

  speed = map(speed, 0, 255, 70, 255);

  // } else if ( speed >= speedThres ) {
  //   // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  //   speed = speedMin;
  // } else if ( speed < speedThres ) {
  //   // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  //   speed = 0;
  // }

  if (u<0){
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  }
  analogWrite(enaPin, speed); // Send PWM signal to motor A
}

// ===================================================================
void loop(){

  // Record the time
  currentMillis = millis();

  // Setpoint
  int targetL = -0;
  int targetR = 0;


  //Calculate revolutions per minute
  if (currentMillis - previousMillis > interval) {

    rpmR = ( (float)encoderRightCount*1000*60 / ENC_COUNT_REV)/(currentMillis - previousMillis);
    rpmL = ( (float)encoderLeftCount *1000*60 / ENC_COUNT_REV)/(currentMillis - previousMillis);

    currentRpmR += alpha * (rpmR - currentRpmR);
    currentRpmL += alpha * (rpmL - currentRpmL);

    // PID
    ur =  pidController(currentRpmR, targetR, kp, kd, ki, eIntegralR, ePreviousR, previousTimeR);
    ul =  pidController(currentRpmL, targetL, kp, kd, ki, eIntegralL, ePreviousL, previousTimeL);



    // Resett
    encoderRightCount = 0;
    encoderLeftCount = 0;
    previousMillis = currentMillis;  
  }

  // Control Motor
  moveMotor(in1, in2, enA,  ur);
  moveMotor(in3, in4, enB,  ul);


  Serial.print(targetL);
  Serial.print(", ");
  Serial.print(currentRpmR);
  Serial.print(", ");
  Serial.print(currentRpmL);

  // Serial.print(", ");
  // Serial.print(encoderRightCount);
  // Serial.print(", ");
  // Serial.print(encoderLeftCount);

  //Compass
  //compass.read();
  //int heading = compass.getAzimuth();

  //erial.print(", ");
  //Serial.print(heading);
  // Serial.print(", ");
  // Serial.print(fabs(ur)); // 0 = N, 4 = E, 8 = S, 12 = W
  // Serial.print(", ");
  // Serial.print(fabs(ul)); // 0 = N, 4 = E, 8 = S, 12 = W

  Serial.println();

//   if(myTransfer.available())
//   {

//     // ------------------ Receive Serial ------------------
//     // use this variable to keep track of how many
//     // bytes we've processed from the receive buffer
//     uint16_t recSize = 0;
//     recSize = myTransfer.rxObj(commandStruct, recSize);
    

//     // ------------------ IMU and Compass ------------------

//     long heading;
//     // Read compass values
//     compass.read();

//     // Return Azimuth reading
//     heading = compass.getAzimuth();

//     sensorStruct.heading = float(heading);
//     sensorStruct.ax = float(motorSpeedB);
    
    

//     // ------------------ Send Serial ------------------
//     uint16_t sendSize = 0;
//     sendSize = myTransfer.txObj(sensorStruct, sendSize);
//     myTransfer.sendData(sendSize);
//   }


}