/**
 * @file Teensy.ino
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @brief File containing the execution code for the teensy embedded within the adjustable trim tab
 * @version 0.2
 * @date 2020-11-18
 * @copyright Copyright (c) 2020 
 */

// Programmer defined files
#include "TrimTabMessages.pb.h"
#include "Constants.h"

// Libraries
#include <WiFiEsp.h>                          //driver code for ESP8266 module
#include "SoftwareSerial.h"                   //driver code for converting ESP8266 to Serial output
#include <Servo.h>                            //driver code for operating servo
#include <IPAddress.h>                        //library to assist in IP addressing (primarily to connect the teensy to the Jetson)
#include <ArduinoJson.h>                      //library to assist in the serialization and deserialization of JSON documents

// ESP8266 buffers 
uint8_t rx_buffer[32];                        // receive buffer
unsigned char tx_buffer[32];                  // transmission buffer

// Wifi variables
SoftwareSerial ESPSerial(RX2pin, TX2pin);     // RX2, TX2
char ssid[] = "sailbothot";                   // Name of the hull network
char pass[] = "salad123";                     // Password to hull network
int status = WL_IDLE_STATUS;                  // the Wifi radio's status
WiFiEspClient client;                         // communication between jetson server and teensy client
bool connection = false;                      // connection indicator
volatile int count = 0;                       // count to have leds blink

// Control variables
volatile int ledState = LOW;
volatile unsigned long blinkCount = 0;        // use volatile for shared variables
volatile int vIn = 0;                         // Battery voltage
volatile int retryCount = 0;
IntervalTimer LEDtimer;
IntervalTimer servoTimer;
volatile int missed_msgs = 0;
Servo servo;
volatile float windAngle;                     // Mapped reading from wind direction sensor on the front of the sail
volatile int32_t control_angle;
bool readingNow = false;

IPAddress testAddr = IPAddress(10,42,0,1);

int timeSinceLastComm = 0;
int timeout = 1000;

TrimState_TRIM_STATE state = TrimState_TRIM_STATE_MIN_LIFT;

void setup()
{
  // Show that we're on
  pinMode(powerLED, OUTPUT);
  digitalWrite(powerLED, HIGH);

  Serial.begin(115200);
  // initialize serial for ESP module
  ESPSerial.begin(115200);
  // initialize ESP module
  WiFi.init(&ESPSerial);
  establishConnection();

  pinMode(vInPin, INPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(wifiLED, OUTPUT);
  pinMode(onBoardLED, OUTPUT);

  servo.attach(servoPin);
  servo.write(SERVO_CTR);

  LEDtimer.begin(blinkState, 916682);

  servoTimer.begin(servoControl, 50000);
}

void loop()
{
  vIn = analogRead(vInPin);
  simpleSendReceive();
}

void simpleSendReceive()
{
  if (!connection)
  {
    client.stop();
    establishConnection();
  } else {
    if(!readingNow){
      writeJson();
    }
    if(readingNow){
      readJson();
    }3
  }
  delay(3000);
}

/* 
 * @author Connor Burri
 */
void readJson(){
  // determines if there is data available to receive
  int isData = client.available() > 0 ? 1 : 0;
  int bytes = client.available();

  // reads the data from the ESP8266
  if(isData){
    for(int i = 0; i < bytes; i++){
      rx_buffer[i] = (uint8_t)client.read();
    }
  }

  // getting the size of the data read from the ESP8266
  size_t rx_buffer_size = sizeof(rx_buffer)/sizeof(rx_buffer[0]);

  for(int i = 0; i< rx_buffer_size; i++){
    Serial.print(rx_buffer[i]);
  }
  Serial.println("");

  // constructing the JSON to be stored on the stack; this is data is coming from the jetson
  DynamicJsonDocument responseJSON(256);

  // converting the data from the ESP8266 to a JSON document and storing it
  DeserializationError err = deserializeJson(responseJSON, rx_buffer, rx_buffer_size);

  // checking to make sure that deserialization performed as expected
  if(err){
    Serial.println("DESERIALIZATION FAILED");
    Serial.println(err.c_str());
  }
  else
  {
    const char* con = responseJSON["connor"];
    Serial.println("DESERIALIZATION WORKED");
    Serial.println(con);
  }

  
//  control_angle = (controlAngle.control_angle-980)*PWMScaler;
//  state = controlAngle.state;
//
//  missed_msgs = 0;
    readingNow = false;
    timeSinceLastComm = millis();
}

void writeJson(){
  //prepare the JSON document
  DynamicJsonDocument json(256);
  
  //assigning the data
  json["relative_wind_dir"] = 1; //windAngle; 

  //string representation of the JsonDocument
  String jsonString;

  //serializing the JsonDocument to a JsonString
  size_t bytesSerialized = serializeJsonPretty(json, jsonString);

  //checking that the data was serialized
  if(bytesSerialized){
    //print what is being sent beforehand
    Serial.println("Sending:");
    Serial.println(jsonString);

    //send the message over the ESP8266
    for(int i = 0; i < jsonString.length(); i++){
      tx_buffer[i] = jsonString.charAt(i);
    }
    int dataWritten =  client.write(tx_buffer, bytesSerialized);

    //check to make sure the data was sent to the server
    if(dataWritten){
      readingNow = true;
      timeSinceLastComm = millis();
    }
    else{
      Serial.println("ERROR: ESP8266 FAILED TO SEND TO SERVER");
    }
    
  }
 else{
    Serial.println("ERRORFailed to Serialize the data");
  }
}


void establishConnection()
{
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true)
      ;
  }
  // attempt to connect to WiFi network
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  // you're connected now, so print out the data
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
//  if (client.connect(hullIP, 50000))
  if (client.connect(testAddr, 50000))
  {
    Serial.println("Connected to server!!!");
    connection = true;
  }
  else{
    Serial.println("Failed to connect to server!");
  }
  Serial.print("Listening on port ");
  Serial.println(50000);
  missed_msgs = 0;
  retryCount = 0;
}

void clearConnection()
{
  WiFi.disconnect();
  establishConnection();
}

/**
 * @brief Sets the angle of the servo based on the angle of attack read from the encoder at the front
 * 
 */
void servoControl()
{
  // Read, format, and scale angle of attack reading from the encoder
  windAngle = analogRead(potPin) - POT_HEADWIND;                                            // reads angle of attack data and centers values on headwind
  windAngle = windAngle < 0 ? POT_HEADWIND + windAngle + (1023 - POT_HEADWIND) : windAngle; // wraps angle around
  windAngle = windAngle / 1023.0 * 360.0 - 180;                                             // Convert to degrees, positive when wind from 0-180, negative when wind 180-359

  // Set debug LEDs to on to indicate servo control is active
  digitalWrite(led1Pin, HIGH);
  digitalWrite(led2Pin, HIGH);

  // Write servo position to one read from the Arduino
  //  servo.write(SERVO_CTR + control_angle - 200 - 90);

  switch(state){
    case TrimState_TRIM_STATE_MAX_LIFT_PORT:
      //if the lift angle isnt enough and the heel angle isnt too much the angle of attack is increased
      if ((MAX_LIFT_ANGLE > windAngle+1)) {  //&& (abs(heelAngle) <= maxHeelAngle))) {
        if (control_angle >= 55) { }
        else {
          control_angle++;
        }
      }
    
      //if the lift angle is too much or the max heel angle is too much the sail lightens up
      else if ((MAX_LIFT_ANGLE < windAngle)) {  //&& (abs(heelAngle) <= maxHeelAngle)) || (abs(heelAngle) >= maxHeelAngle)) {
        if (control_angle <= -55) {  }
        else {
          control_angle--;
        }
      }
      servo.write(SERVO_CTR + control_angle - 200 - 90);
      break;
    case TrimState_TRIM_STATE_MAX_LIFT_STBD:
      windAngle*=-1;
      //if the lift angle isnt enough and the heel angle isnt too much the angle of attack is increased
      if ((MAX_LIFT_ANGLE > windAngle+1)) {  //&& (abs(heelAngle) <= maxHeelAngle))) {
        if (control_angle >= 55) { }
        else {
          control_angle++;
        }
      }
    
      //if the lift angle is too much or the max heel angle is too much the sail lightens up
      else if ((MAX_LIFT_ANGLE < windAngle)) {  //&& (abs(heelAngle) <= maxHeelAngle)) || (abs(heelAngle) >= maxHeelAngle)) {
        if (control_angle <= -55) {  }
        else {
          control_angle--;
        }
      }
      servo.write(SERVO_CTR + control_angle - 200 - 90);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_PORT:
      servo.write(SERVO_CTR - 55);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_STBD:
      servo.write(SERVO_CTR + 55);
      break;
    case TrimState_TRIM_STATE_MIN_LIFT:
      servo.write(SERVO_CTR);
      break;
    case TrimState_TRIM_STATE_MANUAL:
      servo.write(control_angle);
      break;
    default:
      servo.write(control_angle);
      break;
  }
}

void blinkState()
{
  // Toggle state
  ledState = ledState == LOW ? HIGH : LOW;

  digitalWrite(powerLED, ledState);

  // Blink WiFi led
  if (!connection)
  {
    digitalWrite(wifiLED, !ledState);
  }
  else
  {
    digitalWrite(wifiLED, HIGH);
  }

  switch(state){
    case TrimState_TRIM_STATE_MAX_LIFT_PORT:
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, LOW);
      break;
    case TrimState_TRIM_STATE_MAX_LIFT_STBD:
      digitalWrite(led1Pin, ledState);
      digitalWrite(led2Pin, LOW);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_PORT:
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, HIGH);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_STBD:
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, ledState);
      break;
    case TrimState_TRIM_STATE_MIN_LIFT:
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, LOW);
      break;
    case TrimState_TRIM_STATE_MANUAL:
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      break;
    default:
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      break;
  }
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
