/*  Arduino DC Motor Control - PWM | H-Bridge | L298N
         Example 02 - Arduino Robot Car Control
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

// ---------------------------------
RF24 radio(10, 3);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

struct Data_Package {
    byte j1PotX;
    byte j1PotY;
    byte j1Button;
    byte j2PotX;
    byte j2PotY;
    byte j2Button;
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
float desired_angle = 0;
float elapsedTime, time, timePrev;
int i;
#define rad_to_deg 180/3.141592654

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float PID, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

/////////////////PID CONSTANTS/////////////////
double kp=3.25;//3.55
double ki=0.04;//0.003
double kd=1.05;//2.05

// ==========================================
void setup() {

  // I2C
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  //Serial.begin(115200);

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
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
}
// ==========================================
void loop() {

  // ---------------------------------------------------------------------------------
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 

/*The tiemStep is the time that elapsed since the previous loop. 
 * This is the value that we will use in the formulas as "elapsedTime" 
 * in seconds. We work in ms so we haveto divide the value by 1000 
 to obtain seconds*/

/*Reed the values that the accelerometre gives.
 * We know that the slave adress for this IMU is 0x68 in
 * hexadecimal. For that in the RequestFrom and the 
 * begin functions we have to put this value.*/
 
   Wire.beginTransmission(0x68);
   Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,6,true); 

   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
  Acceleration_angle[1] = atan2(-1*(Acc_rawZ/16384.0), Acc_rawX/16384.0)*rad_to_deg;
 
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
   
  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0; 
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;

  /*---X axis angle---*/
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];


  error = Total_angle[1] - desired_angle;
   
  pid_p = kp*error; 
  if(-3 <error <3)  pid_i = pid_i+(ki*error);
  pid_d = kd*((error - previous_error)/elapsedTime);

  PID = (pid_p + pid_i + pid_d);

  if ( PID > 255 ) PID = 255;
  if ( PID < -255) PID = -255;

  previous_error = error; //Remember to store the previous error.

//  Serial.print(desired_angle);
//  Serial.print(" ");
//  Serial.print(Total_angle[1]);
//  Serial.print(" ");
//  Serial.print(PID);
//  Serial.println("");

  if ( PID > 0) {
    setForward();
    motorSpeedA = PID;
    motorSpeedB = PID;
  } else {
    setBackward();
    motorSpeedA = abs(PID);
    motorSpeedB = abs(PID);
  };
    
  if (motorSpeedA > 255) motorSpeedA = 255;
  if (motorSpeedA < 0) motorSpeedA = 0;

  if (motorSpeedB > 255) motorSpeedB = 255;
  if (motorSpeedB < 0) motorSpeedB = 0;
  
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  
}

// ==========================================
void setForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void setBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
// ==========================================
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 0;
  data.j2Button = 0;
}
