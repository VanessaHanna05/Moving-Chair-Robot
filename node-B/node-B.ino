/* Code for NODE B */

#include <SPI.h>
#include "RF24.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 */
RF24 radio(9, 10);
/**********************************************************/

byte node_A_address[6] = "NodeA";
byte node_B_address[6] = "NodeB";
float received_distances[3] = {0,0,0};
float obstacle1 = 0;
float obstacle2 = 0;
float obstacle3 = 0;


void setup() {
  Serial.begin(9600);

  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(node_A_address);
  radio.openReadingPipe(1, node_B_address);

  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  
received_distances();
send_to_python();

}

void receive_distances(){
  float distances_received[3];
    if (radio.available()) {
    // Receive an array of floats (three distances)
    if (radio.read(&distances_received, sizeof(distances_received))) {
      // Process the received data
      obstacle1 = distances_received[0];
      obstacle2 = distances_received[1];
      obstacle3 = distances_received[2];
      Serial.println(osbatcle1);
      Serial.println(osbatcle2);
      Serial.println(osbatcle3);
    }
    else{
      Serial.println("nothing is being received i might be sending data");
    }
}
}

void send_to_mega(){
  radio.stopListening();
  float value_to_send;
  if (!radio.write(&value_to_send, sizeof(value_to_send))) {
    Serial.println(F("Failed to send value"));
  }
Serial.println("value successfully sent");
}

void send_to_python(float dist1, float dist2, float dist3){
  String dataToSend = String(dist1) + "," + String(dist2)+","+String(dist3);
  Serial.println(dataToSend);
 
}