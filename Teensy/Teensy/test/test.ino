//#include "pb.h"
//#include "pb_common.h"
//#include "pb_encode.h"
//#include "pb_decode.h"
#include "TrimTabMessages.pb.h"
#include "SoftwareSerial.h"
#include "Constants.h"
#include <Servo.h>

//instantiating the servo object
Servo servo;

//test number variable
int testNumber = 0;

// potentiometer reading
float potentiometerReading;

void setup() {
  //setup the serial output for the test
  Serial.begin(9600);
  
  //setup the LED console
  pinMode(powerLED, OUTPUT);  //furthest left     (Red LED)
  pinMode(wifiLED, OUTPUT);   //second leftmost   (Yellow LED)
  pinMode(led1Pin, OUTPUT);   //second rightmost  (White LED)
  pinMode(led2Pin, OUTPUT);   //furthest right    (White LED)

  //setup wind sensor
  pinMode(potPin, INPUT);

  //setup servo
  servo.attach(servoPin);

  //Countdown to begin test
  countdown(5);
  testNumber++;

  // --- BEGINNING OF LED TESTING --- //
  
  //test power LED
  countdown(3);
  testLED(powerLED);
  testNumber++;

  //test wifi LED
  countdown(3);
  testLED(wifiLED);
  testNumber++;

  //test LED1
  countdown(3);
  testLED(led1Pin);
  testNumber++;

  //test LED2
  countdown(3);
  testLED(led2Pin);
  testNumber++;

  // --- BEGINNING OF SERVO TESTING --- //
  int servoAngle = 0;
  if(servo.attached()){
    while(servoAngle <= 180){
      countdown(3);
      Serial.println("EXPECTED: " + String(servoAngle) + " DEGREES");
      Serial.println("ACTUAL: " + String(testServo(servoAngle)) + " DEGREES");
      testNumber++;
      servoAngle+=45;
    }
  }
  else{
    Serial.println("THE SERVO IS NOT ATTACHED");
  }
  testNumber++;

  // --- BEGINNING OF POTENTIOMETER TESTING --- //
  countdown(3);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  potentiometerReading = analogRead(potPin);
  Serial.println((float)(potentiometerReading/1023*360), 2);
}

//function to test an LED
void testLED(int ledPin){
  for(int i = 0; i < 3; i++){
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

int testServo(int angle){
  servo.write(angle);
  return servo.read();
}

//function to initiate a countdown to the serial monitor
void countdown(int seconds){
  //write which test we are performing
  switch(testNumber){
    case 10:
      Serial.println("TESTING POTENTIOMETER IN...\n");
      break;
    case 9:
      Serial.println("TESTING SERVO - 180 DEGREES - IN...\n");
      break;
    case 8:
      Serial.println("TESTING SERVO - 135 DEGREES - IN...\n");
      break;
    case 7:
      Serial.println("TESTING SERVO - 90 DEGREES - IN...\n");
      break;
    case 6:
      Serial.println("TESTING SERVO - 45 DEGREES - IN...\n");
      break;
    case 5:
      Serial.println("TESTING SERVO - 0 DEGREES - IN...\n");
      break;
    case 4:
      Serial.println("TESTING LED 2 IN...\n");
      break;
    case 3:
      Serial.println("TESTING LED 1 IN...\n");
      break;
    case 2:
      Serial.println("TESTING WIFI LED IN...\n");
      break;
    case 1:
      Serial.println("TESTING POWER LED IN...\n");
      break;
    default:
      Serial.println("BEGINNING TESTING IN...\n");
      break;
  }

  //countdown until start of test
  while(seconds > 0){
    Serial.println(seconds);
    seconds--;
    delay(1000);
  }

  //tell the tester to move the potentiometer
  if(testNumber == 10){
    Serial.println("MOVE THE POTENTIOMETER");
    delay(3000);
  }
}
