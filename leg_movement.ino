#include <Servo.h>

Servo servo1;
Servo servo2;

int joy1Pin = A0;  // analog pin used to connect the joystick 1
int joy2Pin = A1;  // analog pin used to connect the joystick 2

void setup() 
{ 
  servo1.attach(3);  // attaches the servo on pin 9
  servo2.attach(5); // attaches the servo on pin 10
} 

void loop() 
{ 
  int joy1Val = analogRead(joy1Pin); 
  int joy2Val = analogRead(joy2Pin); 

  // map the joystick values to servo values
  
  int servo1Val = map(joy1Val, 0, 1023, 0, 100); 
  int servo2Val = map(joy2Val, 0, 1023, 0, 100); 

  // move the servos
  servo1.write(servo1Val); 
  servo2.write(servo2Val); 

  delay(15);   
} 




