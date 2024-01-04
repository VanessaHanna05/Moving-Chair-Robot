int value1 = 42;
float value2 = 3.14;

void setup() {
  Serial.begin(115200); // Set the baud rate
}

void loop() {
  
  // Combine the two values into a single string with a delimiter (e.g., a comma)
  String dataToSend = String(value1) + "," + String(value2);
  Serial.println(dataToSend);
  delay(100); // Delay for a second before sending again
}
