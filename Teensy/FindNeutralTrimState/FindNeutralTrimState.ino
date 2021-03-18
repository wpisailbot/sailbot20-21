/**
 * @file FindNeutralTrimState.ino
 * @author Irina Lavryonova (ilavyonova@wpi.edu)
 * @brief Code that makes it simple to find the wind vein and trim tab center positions, useful for calibrating both
 * @version 0.1
 * @date 2019-10-05
 */
#include <Servo.h>

//Pins for devices
#define servoPin 6
#define potpin 38
//Servo settings
#define lowerLimit 30
#define middle 115
#define upperLimit 180
//Pot settings
#define headwind 463
#define clockwise true 

Servo servo;

String readString = "";

int potAngle;

void setup()
{
    Serial.begin(115200);
    servo.attach(servoPin);
}

void loop()
{
    /*   Use below code to find wind vein center and direction   */
//    potAngle = analogRead(potpin);
//    Serial.print("  Wind Vein Angle:");
//    Serial.println(potAngle);

    /*   Use below code to find trim tab center and bounds   */
     while (Serial.available()) {
         char c = Serial.read();  //gets one byte from serial buffer
         readString += c; //makes the string readString
         delay(2);  //slow looping to allow buffer to fill with next character
     }

     if (readString.length() >0) {
         Serial.println(readString);  //so you can see the captured string
         int n = readString.toInt();  //convert readString into a number

         servo.write(n);

         readString=""; //empty for next input
     }
}
