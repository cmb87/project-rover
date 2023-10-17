/*  Arduino DC Motor Control - PWM | H-Bridge | L298N
         Example 02 - Arduino Robot Car Control
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#include <SPI.h>
#include <RF24.h>


// ---------------------------------
RF24 radio(10, 3);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

struct Data_Package {
    byte pitch;
    byte yaw;
    byte throttle;
    byte roll;
    byte aux1;
    byte aux2;
    byte aux3;
    byte aux4;
};

Data_Package data; //Create a variable with the above structure

// ---------------------------------

// https://lastminuteengineers.com/stepper-motor-l298n-arduino-tutorial/
// https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/

#define enA 4 //D4
#define in1 5 //D5
#define in2 6 //D6
#define in3 7 //D7
#define in4 8 //D8
#define enB 9 //D9

#define IRSensorR 15 // A6 connect ir sensor to arduino pin 2
#define IRSensorL 2 // D2 connect ir sensor to arduino pin D2

int motorSpeedA = 0;
int motorSpeedB = 0;

bool autpilotFlag = false;

// ==========================================
void setup() {

  Serial.begin(115200);

  // IR Sensor
  pinMode (IRSensorR, INPUT); // sensor pin INPUT
  pinMode (IRSensorL, INPUT); // sensor pin INPUT
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Start radio
  radio.begin();
  radio.openReadingPipe(0, address);
  //radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening(); //  Set the module as receiver
  
}
// ==========================================
void loop() {
//  directionControl();
//  delay(1000);
//  speedControl();
//  delay(1000);

  int statusSensorR = digitalRead(IRSensorR);
  int statusSensorL = digitalRead(IRSensorL);


  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    Serial.println("Connection lost :(");
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data

    Serial.print(data.pitch);
    Serial.print(" ");
    Serial.print(data.yaw);
    Serial.print(" ");
    Serial.print(data.throttle);
    Serial.print(" ");
    Serial.print(data.roll);
    Serial.print(" ");
    Serial.print(data.aux1);
    Serial.print(" ");
    Serial.print(data.aux2);
    Serial.print(" ");
    Serial.print(data.aux3);
    Serial.print(" ");
    Serial.print(data.aux4);
    Serial.println();
  }

  
  if (data.aux1 == 1 && autpilotFlag) {
    autpilotFlag = false;
  } else if (data.aux1 == 1 && !autpilotFlag) {
    autpilotFlag = true;
  }
  
  if (data.throttle > 127) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    
    motorSpeedA = map(data.throttle, 127, 255, 0, 255) - map(data.yaw, 0, 255, -255, 255);
    motorSpeedB = map(data.throttle, 127, 255, 0, 255) + map(data.yaw, 0, 255, -255, 255);

  } else if (data.throttle < 127) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    
    motorSpeedA = map(data.yaw, 127, 0, 0, 255) - map(data.yaw, 0, 255, -255, 255);
    motorSpeedB = map(data.yaw, 127, 0, 0, 255) + map(data.yaw, 0, 255, -255, 255);

  } else if (data.yaw != 127) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    
    motorSpeedA = 127-map(data.throttle, 0, 255, -255, 255);
    motorSpeedB = 127+map(data.throttle, 0, 255, -255, 255);
    
  } else if (autpilotFlag) {


    
    if (statusSensorR == 1 && statusSensorL == 1) {

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      
      motorSpeedA = 255;
      motorSpeedB = 255;
    } else if (statusSensorR == 0 && statusSensorL == 1) {

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    
      motorSpeedA = 100;
      motorSpeedB = 255;
      
    } else if (statusSensorR == 1 && statusSensorL == 0) {

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    
      motorSpeedA = 255;
      motorSpeedB = 100;
    } else {
      
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    
      motorSpeedA = 255;
      motorSpeedB = 0;
    
    }
    
  } else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    motorSpeedA = 0;
    motorSpeedB = 0;
  }



  if (motorSpeedA > 255) motorSpeedA = 255;
  if (motorSpeedA < 0) motorSpeedA = 0;

  if (motorSpeedB > 255) motorSpeedB = 255;
  if (motorSpeedB < 0) motorSpeedB = 0;
//
//
//  Serial.print(motorSpeedA);
//  Serial.print(" ");
//  Serial.println(motorSpeedB) ;
  
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  
}
// ==========================================
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.pitch = 1500;
  data.yaw = 1500;
  data.throttle = 1000;
  data.roll = 1500;
  data.aux1 = 0;
  data.aux2 = 0;
  data.aux3 = 0;
  data.aux4 = 0;
}
