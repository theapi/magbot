/**
 PIN ALLOCATION:
 
 13 - Motor B direction
 12 - Motor A direction
 11 - Motor B pwm
 9  - Motor A brake
 8  - Motor B brake
 7  - Ping trigger
 3  - Motor A pwm
 4  - Ping echo
*/

#include <NewPing.h>

#define MOTOR_SPEED_MIN_A 200
#define MOTOR_SPEED_MAX_A 255 
#define MOTOR_SPEED_MIN_B 200
#define MOTOR_SPEED_MAX_B 235 // This motor is faster, so slow it down.


// Right Motor attached to channel A:
const byte motorA_direction = 12;
const byte motorA_pwm = 3; 
const byte motorA_brake = 9;

// Left Motor attached to channel B:
const byte motorB_direction = 13;
const byte motorB_pwm = 11; 
const byte motorB_brake = 8;


// Ping configuration
const byte ping_trigger = 7;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
const byte ping_echo = 4;     // Arduino pin tied to echo pin on the ultrasonic sensor.
const int ping_distance = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(ping_trigger, ping_echo, ping_distance); // NewPing setup of pins and maximum distance.


void setup() 
{
  
  Serial.begin(9600); // Open serial monitor at 9600 baud to see debugging messages.
  
  // Setup Channel A
  pinMode(motorA_direction, OUTPUT); // Initiates motor pin for channel A
  pinMode(motorA_brake, OUTPUT);     // Initiates break pin for channel A

  // Setup Channel B
  pinMode(motorB_direction, OUTPUT); // Initiates motor pin for channel B
  pinMode(motorB_brake, OUTPUT);     // Initiates break pin for channel B
  
  // Ensure the brakes are off
  digitalWrite(motorA_brake, LOW);   // Disengage the Brake for Channel A
  digitalWrite(motorB_brake, LOW);   // Disengage the Brake for Channel B

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
  analogWrite(motorA_pwm, 0); // Channel A at max speed 
  analogWrite(motorB_pwm, 0); // Channel B at max speed 
}

void motionFwd(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("FORWARD");
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, HIGH); // Channel B forward
  analogWrite(motorA_pwm, speed_a); // Channel A at max speed 
  analogWrite(motorB_pwm, speed_b); // Channel B at max speed 
}

void motionRev(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("REVERSE");
  digitalWrite(motorA_direction, LOW); // Channel A backward
  digitalWrite(motorB_direction, LOW); // Channel B backward
  analogWrite(motorA_pwm, speed_a); // Channel A at minimum speed 
  analogWrite(motorB_pwm, speed_b); // Channel B at minimum speed   
}

void motionRotateLeft(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("LEFT");
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, LOW); // Channel B backward
  analogWrite(motorA_pwm, speed_a);
  analogWrite(motorB_pwm, speed_b);   
}

void motionRotateRight(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("RIGHT");
  digitalWrite(motorA_direction, LOW); // Channel A backward
  digitalWrite(motorB_direction, HIGH); // Channel B forward
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

