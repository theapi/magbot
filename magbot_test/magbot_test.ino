/**
 HC-SR04 Ultrasonic Range Finder attached to the servo.
 Two motors attached to the motor shield.
 */

#include <NewPing.h> // From https://code.google.com/p/arduino-new-ping/
#include <Servo.h>

// Motor attached to channel A:
const byte motorA_direction = 12;
const byte motorA_pwm = 3;
const byte motorA_brake = 9;
const byte motorA_sensor = A0;
// Motor attached to channel B:
const byte motorB_direction = 13;
const byte motorB_pwm = 11;
const byte motorB_brake = 8;
const byte motorB_sensor = A1;

// Sonar configuration
const byte sonar_trigger = 7;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
const byte sonar_echo = 4;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const int sonar_distance = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const byte sonar_delay = 50; // Number of milliseconds to wait before next ping
unsigned long sonar_last = 0; // When the last ping happened.
NewPing sonar(sonar_trigger, sonar_echo, sonar_distance); // NewPing setup of pins and maximum distance.

// Servo configuration
const byte servo_pin = 6;
byte servo_sweep_direction = 0; // The current direction the servo is sweeping; 0 or 1
const byte servo_delay = 25; // Number of milliseconds to wait before next move
unsigned long servo_last = 0; // When the last move happened.
Servo servo;  // create servo object to control a servo

unsigned long millis_now;

void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  
  servo.attach(servo_pin);  // attaches the sonar servo to the servo object 
  
    // Setup Channel A
  pinMode(motorA_direction, OUTPUT); // Initiates motor pin for channel A
  pinMode(motorA_brake, OUTPUT); // Initiates break pin for channel A

  // Setup Channel B
  pinMode(motorB_direction, OUTPUT); // Initiates motor pin for channel B
  pinMode(motorB_brake, OUTPUT);  // Initiates break pin for channel B
  
  // Ensure the brakes are off
  digitalWrite(motorA_brake, LOW);   // Disengage the Brake for Channel A
  digitalWrite(motorB_brake, LOW);   // Disengage the Brake for Channel B
  
  
  // Turn in place 
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, LOW); // Channel B backward
  
  analogWrite(motorA_pwm, 255); // Channel A at max speed 
  analogWrite(motorB_pwm, 255); // Channel B at max speed
}

void loop() {
  millis_now = millis();
  
  // Check to see if it's time for another ping.
  if (millis_now - sonar_last > sonar_delay) {
    // Remeber when this ping was done, so we know when to do the next one.
    sonar_last = millis_now;
  
    unsigned int cm = sonar.ping_cm(); // Send a ping, get ping time in microseconds (uS).
    Serial.print("Ping: ");
    Serial.print(cm);
    Serial.println("cm");
  }
  
  // Check to see if it's time for another soanr servo move.
  if (millis_now - servo_last > servo_delay) {
    // Remember when this move was done, so we know when to do the next one.
    servo_last = millis_now;
    
    // Sweep
    // Get the last angle the servo was set to.
    int servo_pos = servo.read();
    if (servo_sweep_direction == 0) {
      servo_pos++; // Add 1 degree to the servo's position.
    } else {
      servo_pos--; // Remove 1 degree to the servo's position.
    }
    
    // At the end of the sweep, change directions for the next move.
    if (servo_pos == 170) {
      servo_sweep_direction = 1;
    } else if (servo_pos == 10) {
      servo_sweep_direction = 0;
    }
    servo.write(servo_pos); 
  }
  
}

