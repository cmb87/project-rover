    /*
            DIY Arduino based RC Transmitter
      by Dejan Nedelkovski, www.HowToMechatronics.com
      Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
      https://howtomechatronics.com/projects/diy-arduino-rc-transmitter/
    */
    #include <SPI.h>
    #include <nRF24L01.h>
    #include <RF24.h>
    #include <Wire.h>
    
    // Define the digital inputs
    #define jBL 2  // Joystick button 1
    #define jBR 3  // Joystick button 2
    #define jXL 1  // Joystick button 1
    #define jYL 2  // Joystick button 2
    #define jXR 3  // Joystick button 1
    #define jYR 6  // Joystick button 2
    
    float elapsedTime, currentTime, previousTime;
    
    int c = 0;

    int j1PotXraw;
    int j1PotYraw;
    int j2PotXraw;
    int j2PotYraw;
    
    RF24 radio(10, 9);   // nRF24L01 (CE, CSN)
    
    const byte address[6] = "00001"; // Address
    // Max size of this struct is 32 bytes - NRF24L01 buffer limit
    struct Data_Package {
      byte j1PotX;
      byte j1PotY;
      byte j1Button;
      byte j2PotX;
      byte j2PotY;
      byte j2Button;
    };
    
    Data_Package data; //Create a variable with the above structure
    
    void setup() {
      Serial.begin(9600);

      Serial.print("Setting up sender ..\n");
      // Define the radio communication
      radio.begin();
      radio.openWritingPipe(address);
      //radio.setAutoAck(false);
      radio.setDataRate(RF24_250KBPS);
      radio.setPALevel(RF24_PA_LOW);
      radio.stopListening();
      // Activate the Arduino internal pull-up resistors
      //pinMode(jBL, INPUT_PULLUP);
      //pinMode(jBR, INPUT_PULLUP);


      pinMode(jXL, INPUT);
      pinMode(jYL, INPUT);
      pinMode(jBL, INPUT_PULLUP); 
      pinMode(jXR, INPUT);
      pinMode(jYR, INPUT);
      pinMode(jBR, INPUT_PULLUP); 

      
      // Set initial default values
      data.j1PotX = 127; // Values from 0 to 255. When Joystick is in resting position, the value is in the middle, or 127. We actually map the pot value from 0 to 1023 to 0 to 255 because that's one BYTE value
      data.j1PotY = 127;
      data.j2PotX = 127;
      data.j2PotY = 127;
      data.j1Button = 1;
      data.j2Button = 1;
      Serial.print("Done...");
    }

    //--------------------------------------------------------
    void loop() {
      // Read all analog inputs and map them to one Byte value

      j1PotXraw = analogRead(jXL);
      j1PotYraw = analogRead(jYL);
      j2PotXraw = analogRead(jXR);
      j2PotYraw = analogRead(jYR);
      
      data.j1PotX = map(j1PotXraw, 0, 1023, 255, 0); // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
      data.j1PotY = map(j1PotYraw, 0, 1023, 0, 255);
      data.j2PotX = map(j2PotXraw, 0, 1023, 0, 255);
      data.j2PotY = map(j2PotYraw, 0, 1023, 255, 0);

      // Read all digital inputs
      data.j1Button = 1-digitalRead(jBL);
      data.j2Button = 1-digitalRead(jBR);

      // Send the whole data from the structure to the receiver
      radio.write(&data, sizeof(Data_Package));

      // Print the data in the Serial Monitor
      Serial.print("j1PotX: ");
      Serial.print(data.j1PotX);
      Serial.print("; j1PotY: ");
      Serial.print(data.j1PotY);
      Serial.print("; j1But: ");
      Serial.print(data.j1Button);

      Serial.print("; j2PotX: ");
      Serial.print(data.j2PotX);
      Serial.print("; j2PotY: ");
      Serial.print(data.j2PotY);
      Serial.print("; j2But: ");
      Serial.print(data.j2Button);
      Serial.print("\n");
//
//      Serial.print("j1Xr: ");
//      Serial.print(j1PotXraw);
//      Serial.print("; j1Yr: ");
//      Serial.print(j1PotYraw);
//      Serial.print("j2Xr: ");
//      Serial.print(j2PotXraw);
//      Serial.print("; j2Yr: ");
//      Serial.print(j2PotYraw);
//      Serial.print("\n");


      
    }
