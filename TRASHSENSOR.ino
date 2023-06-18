#include <Servo.h> 
int servoPin = 8; 
int IRSensor = 9; // connect IR sensor module to Arduino pin D9
int LED = 7;
int LED1 = 4;
Servo Servo1; 
void setup(){
  Serial.begin(115200); // Init Serial at 115200 Baud Rate.
  Serial.println("Serial Working"); // Test to check if serial is working or not
  pinMode(IRSensor, INPUT); // IR Sensor pin INPUT
  pinMode(LED, OUTPUT); // LED Pin Output
  Servo1.attach(servoPin);
}
void loop(){
  int sensorStatus = digitalRead(IRSensor); // Set the GPIO as Input
  if (sensorStatus == 1) // Check if the pin high or not
  {
    // if the pin is high turn off the onboard Led
    digitalWrite(LED1, HIGH); // LED LOW
    Serial.println("No Motion found"); // print Motion Detected! on the serial monitor window
  }
  else  {
    digitalWrite(LED1, LOW);
    //else turn on the onboard LED
    digitalWrite(LED, HIGH); // LED High
    Serial.println("Motion Detected"); // print Motion Ended! on the serial monitor window
    delay(5000);
  
    Servo1.write(0); 
   
    // Make servo go to 180 degrees 
    Servo1.write(180); 


   
    delay(2000); 
    Servo1.write(0); 
    delay(2000);
    digitalWrite(LED, LOW);
    

   
    

    
  }
  
}