/**
 Created by Peter Clarke

 PIN ALLOCATION:
 
 13 - Motor B direction 1
 12 - Motor A direction 2
 11 - Motor B direction 2
 8  - Motor A direction 1
 7  - Ping trigger
 6  - Motor B pwm
 5  - Motor A pwm
 4  - Ping echo
 
 
*/

// https://code.google.com/p/arduino-new-ping/
#include <NewPing.h>

#define MOTOR_SPEED_MIN_A 200
#define MOTOR_SPEED_MAX_A 255 
#define MOTOR_SPEED_MIN_B 200
#define MOTOR_SPEED_MAX_B 255 


// Right Motor attached to channel A:
const byte motorA_direction1 = 8;
const byte motorA_direction2 = 12;
const byte motorA_pwm = 5; 

// Left Motor attached to channel B:
const byte motorB_direction1 = 13;
const byte motorB_direction2 = 11;
const byte motorB_pwm = 6; 


// Ping configuration
const byte ping_trigger = 7;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
const byte ping_echo = 4;     // Arduino pin tied to echo pin on the ultrasonic sensor.
const int ping_distance = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(ping_trigger, ping_echo, ping_distance); // NewPing setup of pins and maximum distance.


void setup() 
{
  
  Serial.begin(9600); // Open serial monitor at 9600 baud to see debugging messages.
  
  // Setup Channel A
  pinMode(motorA_direction1, OUTPUT); // Initiates motor pin for channel A
  pinMode(motorA_direction2, OUTPUT); // Initiates motor pin for channel A
  pinMode(motorA_pwm, OUTPUT);        // Initiates speed control for channel A 

  // Setup Channel B
  pinMode(motorB_direction1, OUTPUT); // Initiates motor pin for channel B
  pinMode(motorB_direction2, OUTPUT); // Initiates motor pin for channel B
  pinMode(motorB_pwm, OUTPUT);        // Initiates speed control for channel B 
  

  // Start moving
  motionFwd(255);
}

void loop() 
{
  delay(100);
  pingMeasure();
}

/********************************************************************************
Ping functions
********************************************************************************/

void pingMeasure()
{
  unsigned int cm = sonar.ping_cm(); // Send a ping, get ping distance in cm.
  if (cm > 0 && cm < 20) {
    Serial.println("Too close!");
    
    // Reverse for a second.
    motionRev(220);
    delay(1000);
    
    // Turn right for half a second
    motionRotateRight(220);
    delay(500);
    
    // Carry on
    motionFwd(255);
  }
}


/********************************************************************************
Motion functions
********************************************************************************/

void motionStop()
{
  Serial.println("STOP");
  analogWrite(motorA_pwm, 0); // Zero speed 
  analogWrite(motorB_pwm, 0); // Zero speed 
}

void motionFwd(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("FORWARD");
  digitalWrite(motorA_direction1, HIGH); // Channel A forward
  digitalWrite(motorA_direction2, LOW);
  
  digitalWrite(motorB_direction1, HIGH); // Channel B forward
  digitalWrite(motorB_direction2, LOW); 
  
  analogWrite(motorA_pwm, speed_a); // set the speed 
  analogWrite(motorB_pwm, speed_b); // set the speed 
}

void motionRev(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("REVERSE");
  
  digitalWrite(motorA_direction1, LOW); // Channel A reverse
  digitalWrite(motorA_direction2, HIGH);
  
  digitalWrite(motorB_direction1, LOW); // Channel B reverse
  digitalWrite(motorB_direction2, HIGH); 
  
  analogWrite(motorA_pwm, speed_a); // set the speed 
  analogWrite(motorB_pwm, speed_b); // set the speed 
}

void motionRotateLeft(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("LEFT");
  
  digitalWrite(motorA_direction1, HIGH); // Channel A forward
  digitalWrite(motorA_direction2, LOW);
  
  digitalWrite(motorB_direction1, LOW); // Channel B reverse
  digitalWrite(motorB_direction2, HIGH); 

  analogWrite(motorA_pwm, speed_a);
  analogWrite(motorB_pwm, speed_b);   
}

void motionRotateRight(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("RIGHT");
  
  digitalWrite(motorA_direction1, LOW); // Channel A reverse
  digitalWrite(motorA_direction2, HIGH);
  
  digitalWrite(motorB_direction1, HIGH); // Channel B forward
  digitalWrite(motorB_direction2, LOW); 
  
  analogWrite(motorA_pwm, speed_a);
  analogWrite(motorB_pwm, speed_b);
}

byte motionSpeedCompensateA(byte speed)
{
  
  if (MOTOR_SPEED_MAX_B > MOTOR_SPEED_MAX_A) {
    // Need to slow down this motor
    speed = speed - (MOTOR_SPEED_MAX_B - MOTOR_SPEED_MAX_A);
  }

  if (speed < MOTOR_SPEED_MIN_A) {
    speed = MOTOR_SPEED_MIN_A;
  } else if (speed > MOTOR_SPEED_MAX_A) {
    speed = MOTOR_SPEED_MAX_A;
  }
  
  return speed;
}

byte motionSpeedCompensateB(byte speed)
{
  
  if (MOTOR_SPEED_MAX_A > MOTOR_SPEED_MAX_B) {
    // Need to slow down this motor
    speed = speed - (MOTOR_SPEED_MAX_A - MOTOR_SPEED_MAX_B);
  }

  if (speed < MOTOR_SPEED_MIN_B) {
    speed = MOTOR_SPEED_MIN_B;
  } else if (speed > MOTOR_SPEED_MAX_B) {
    speed = MOTOR_SPEED_MAX_B;
  }
  
  return speed;
}

