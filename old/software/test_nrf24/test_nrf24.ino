#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
char msg[6];
RF24 radio(10,3);
const uint64_t pipe = 0xE8E8F0F0E1LL;
void setup(void){
 Serial.begin(115200);
 radio.begin();
 radio.setChannel(2);
 radio.setPayloadSize(7);
 radio.setDataRate(RF24_250KBPS);
 radio.openReadingPipe(1,pipe);
 radio.startListening();
}

void loop(void){
 if (radio.available()){  
     radio.read(msg, 6);      
     Serial.println(msg);
     delay(10);
 }
 else{
  Serial.println("No radio available");
 }
 delay(1000);
}
