#include <SPI.h>
#include <Wire.h>

int counter = 0;
bool clap_detector = false;
int check = 0;
unsigned long last_event = 0;
int clap_pin = 46;// pin of the clap detector
int ldr = 0; // the initial state of the LDR 
int  trigPin =  52;
int  echoPin =  53;
int  trigPin2 = 50;
int  echoPin2 = 51;
int  trigPin3 = 48;
int  echoPin3 = 49;
float duration,distance;
float duration2,distance2;
float duration3,distance3;
float obstacleright = 0;
float obstacleforward = 0;
float obstacleleft = 0;
float obstacles[3];
// Setup Motor A (front and rear) pins
int enableA = 9;
int pinA1 = 8;
int pinA2 = 7;
// Setup Motor B (front and rear) pins
int enableB = 10;
int pinB1 = 6;
int pinB2 = 5;
const float targetRSSI = 45;  // to be modified 
const int ObstacleThreshold = 40;   // Threshold distance to consider an obstacle (adjust as needed)
 String direction;
 int prevDistance = 0;
 const int initiralDistance = 0;
 int RSSIValue=0;
 int currentDistance;
unsigned long timeStrored=0;

//=================================================================

void setup() {
  // put your setup code here, to run once:
  pinMode(A1, INPUT); // output of the sensor (LDR)
  pinMode(clap_pin, INPUT);// clapping input from the sound sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3,INPUT);
  //motors setup
  enableMotors() ;
  Serial.begin(9600);
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
  currentDistance = abs(RSSIValue - targetRSSI);
 
     

}
void receiveEvent(int bytes){
  RSSIValue = Wire.read();
}


void loop() {

  if(currentDistance>0 && check==0){
     prevDistance = currentDistance;
     forward();

     delay(500);
     breakRobot();
     check=1;

  }
  counter++;
  if(counter==2){
     prevDistance = currentDistance;
     counter =0;
  }

    ldr = analogRead(A1);
  Serial.println("LDR analog value: " );
  Serial.println(ldr);
  Serial.print("RSSI value: ");
  Serial.println(RSSIValue);
  delay(100);

 
  if(ldr < 900 && RSSIValue!=0)
  {  // the chair can move since there is no one sitting
    
    Serial.println("There is no one on the chair!");
    if(clap_detect()){ // if a clap is detected
    Serial.println("The clap is detected");
    
    ldr = analogRead(A0);
    if(ldr >900){ // set to make sure that if the chair is moving and someone sat on it it should stop from moving
      clap_detector = false;
      Serial.println("someone is sitting on the chair");

      }

    else{// the chair should start moving since the clap is detected and no one is sitting on it
      // obstacle detection algorithm
     
      obstacleright = calculate_distance1();
      delay(100);
      obstacleforward = calculate_distance2();
      delay(100);
     obstacleleft = calculate_distance3();
      delay(100);
     
     
      
      currentDistance = abs(RSSIValue - targetRSSI);
     
      Serial.print("Current distance is: ");
      Serial.println(currentDistance);
      Serial.print("Previous distance is: ");
      Serial.println(prevDistance);
      if(currentDistance<=5){
        Serial.println("Distance reached..................................");
        breakRobot();
        clap_detector = false;
        return;
      }

      if(abs(currentDistance - prevDistance)<=5 || currentDistance< prevDistance){
        // in this case the robot moved towards from the target and should reverse its direction
    if(obstacleforward<ObstacleThreshold){
      //if there is an obstacle in front 
      breakRobot();
      if(obstacleright<ObstacleThreshold){
           // if there is an obstacle to the right and front
        if(obstacleleft<ObstacleThreshold){
             // if there is an obstacle to the right and front and left
          Serial.println("left right front");
          analogWrite(10,200);
          analogWrite(9,200);
          right(1600);
           Serial.println("rotate to the right 2");
          //return;
        }else{
             
          analogWrite(10,200);
          analogWrite(9,200);
        left(900);
        Serial.println(" right front");
        Serial.println("rotate to the left");
        }
       // return;
      }else if(obstacleleft<ObstacleThreshold){
        analogWrite(10,200);
          analogWrite(9,200);
        right(900);
         Serial.println("rotate to the right");
         Serial.println("left front");
        //return;
      }else{
        analogWrite(10,200);
          analogWrite(9,200);
      right(900);
       Serial.println("rotate to the right");
       Serial.println("front");
      //return;
      }
    }
    else if(obstacleforward>ObstacleThreshold){
      //if there is no obstacle in front 
      if(obstacleleft<ObstacleThreshold){
        // if there is an obstacle to the right
        if(obstacleright<ObstacleThreshold){
          // if there is an obstacle to the left 
          analogWrite(10,200);
          analogWrite(9,200);
          right(1800);
           Serial.println("rotate to the right 2");
          Serial.println("left right");
         // return;
        }else{

          analogWrite(10,200);
          analogWrite(9,200);
        right(900);
         Serial.println("rotate to the right");
        Serial.println(" left");
        }
        //return;
      }else if(obstacleright<ObstacleThreshold){
      analogWrite(10,200);
          analogWrite(9,200);
     
        left(900);
         Serial.println("rotate to the left");
        Serial.println("right");
       // return;

      }
      else{
          analogWrite(10,150);
          analogWrite(9,150);

          forward();
      prevDistance =  currentDistance;
      Serial.println("Moving forward");
      return;
      }
    }
      }else{
        // in this case the robot moved away from the target and should reverse its direction
        Serial.println("I moved away from the target");
        analogWrite(10,200);
        analogWrite(9,200);
        right(19000);
        breakRobot();
        delay(1000);
        return;
        // after reversing the direction move normally 
        
      }
     }
    }
  }
}

bool clap_detect() {
  int output = digitalRead(clap_pin);
  if (output == LOW) {
      Serial.println("Clap sound was detected!");
      clap_detector = true;
      return clap_detector;  
  }
  return clap_detector;
}

int calculate_distance1(){

  digitalWrite(trigPin,LOW);
  delay(2);
  digitalWrite(trigPin,HIGH);
  delay(10); //sending a pulse of duration 10 micro seconds to trigger the signal of the sensor
  digitalWrite(trigPin,LOW);

  // measuring the distance from the pulse response
  duration = pulseIn(echoPin,HIGH);

  // DETERMINE THE DISTANCE FRIN DURATION BY USING THAT THE speed of sound is 343 meters/s

  distance = (duration/2)*0.0343;

  Serial.println("distance 1: " + String(distance));
  return distance;
  
}

int calculate_distance2(){

 digitalWrite(trigPin2,LOW);
  delay(2);
  digitalWrite(trigPin2,HIGH);
  delay(10); //sending a pulse of duration 10 micro seconds to trigger the signal of the sensor
  digitalWrite(trigPin2,LOW);

  // measuring thr distance from the pulse response
  duration2 = pulseIn(echoPin2,HIGH);

  // DETERMINE THE DISTANCE FRIN DURATION BY USING THAT THE speed of sound is 343 meters/s

  distance2 = (duration2/2)*0.0343;

  Serial.println("distance 2: " + String(distance2));
  return distance2;
}

int calculate_distance3(){
   digitalWrite(trigPin3,LOW);
  delay(2);
  digitalWrite(trigPin3,HIGH);
  delay(10); //seding a pulse of duration 10 micro seconds to trigger the signal of the sensor
  digitalWrite(trigPin3,LOW);

  // measuring thr distance from the pulse response
  duration3 = pulseIn(echoPin3,HIGH);

  // DETERMINE THE DISTANCE FRIN DURATION BY USING THAT THE speed of sound is 343 meters/s

  distance3 = (duration3/2)*0.0343;

  Serial.println("distance 3: " + String(distance3));
  return distance3;

}


// Create motor functions
void motorAforward() {
   digitalWrite (pinA1, LOW);
 digitalWrite (pinA2, HIGH);

}
void motorBforward() {
  digitalWrite (pinB1, LOW);
 digitalWrite (pinB2, HIGH);
 
}
void motorAbackward() {
 digitalWrite (pinA1, HIGH);
 digitalWrite (pinA2, LOW);
}
void motorBbackward() {
 digitalWrite (pinB1, HIGH);
 digitalWrite (pinB2, LOW);
}
void motorAstop() {
 digitalWrite (pinA1, HIGH);
 digitalWrite (pinA2, HIGH);
}
void motorBstop() {
 digitalWrite (pinB1, HIGH);
 digitalWrite (pinB2, HIGH);
}
void motorAcoast() {
 digitalWrite (pinA1, LOW);
 digitalWrite (pinA2, LOW);
}
void motorBcoast() {
 digitalWrite (pinB1, LOW);
 digitalWrite (pinB2, LOW);
}
void motorAon() {
  analogWrite(9,150);
  
}
void motorBon() {
 analogWrite(10,150);

}
void motorAoff(){
 digitalWrite (enableA, LOW);
}

void motorBoff() {
 digitalWrite (enableB, LOW);
}

// Setup movement functions
void forward () {
 
 motorAforward();
 motorBforward();
}
void backward (int duration) {
 motorAbackward();
 motorBbackward();
 delay (duration);
}
void right (int duration) {
 motorAbackward();
 motorBforward();
 delay (duration);
 breakRobot();
}
void left (int duration) {
 motorAforward();
 motorBbackward();
 delay (duration);
  breakRobot();
}
void coast (int duration) {
 motorAcoast();
 motorBcoast();
 delay (duration);
}
void breakRobot () {
 motorAstop();
 motorBstop();
}
void disableMotors() {
 motorAoff();
 motorBoff();
}
void enableMotors() {
  digitalWrite (pinA1, LOW);
 digitalWrite (pinA2, LOW);
  digitalWrite (pinB1, LOW);
 digitalWrite (pinB2, LOW);
 motorAon();
 motorBon();
}
// This function only runs if an obstacle within 15cm is detected
void avoidLeft()
{
    backward(500);
    right(360);
}
void avoidRight()
{
    backward(500);
    left(360);
}