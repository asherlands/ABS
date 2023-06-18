#include <WiFi.h> 
#include <analogWrite.h>
#include <ESP32Servo.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <Arduino.h>
#include <FirebaseESP32.h>

#define FIREBASE_Host "https://bottle-seg-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_authorization_key "rgubHBEAkgTIJu3D4unxRtkkNGCZkYDKwf2nywPs"
#define Your_SSID "POCO"//FTTH-96C3
#define Your_PASSWORD "123456789"//landsasher123
#define SERVO_PIN 14 // Change to the appropriate GPIO pin for your setup
#define IRSensor 5

Servo servo;
FirebaseData firebaseData;
String servo_state = "";
int servo_gpio = 14;
String path = "/x/bottle/bottle/bottle";
String value;
int numericValue;
bool motionDetected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(servo_gpio, OUTPUT);
  pinMode(IRSensor, INPUT);
  
  WiFi.begin(Your_SSID, Your_PASSWORD);
  Serial.print("Connecting to WIFI");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.print("Connected to WIFI!");
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_Host, FIREBASE_authorization_key);
  servo.attach(SERVO_PIN);
}

void loop() {
  int sensorStatus = digitalRead(IRSensor);

  if (sensorStatus == 1) {
    //digitalWrite(LED1, HIGH);
    Serial.println("No Motion found");
    motionDetected = false; // Set motionDetected to false if no motion is detected
  }
  else {
    //digitalWrite(LED1, LOW);
    Serial.println("Motion detected");
    motionDetected = true; // Set motionDetected to true if motion is detected
  }

  if (motionDetected) {
    delay(1000);
    if (Firebase.getString(firebaseData, path)) {
      if (firebaseData.dataType() == "string") {
        value = firebaseData.stringData();
      }

      if (value == "1") {
      delay(5000);
      //servo.write(90);
      servo.write(180);
      
      delay(2000);
      servo.write(90);
      Serial.println("Servo rotated to 180 degrees");
    }
    else if (value == "0") {
      delay(6000);
      //servo.write(90);
      servo.write(0);
      
      
     delay(2000);
     servo.write(90);
      Serial.println("Servo rotated to 0 degrees");
    }
      else {
        Serial.println("Invalid value received");
      }
    }
  }
  
  delay(100); // Add a small delay between sensor readings
}
