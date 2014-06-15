/**
 HC-SR04 Ultrasonic Range Finder.
 Two motors attached to the motor shield.
 speaker/piezo for sound
 
 Arduino Uno has three timers; timer 0, timer 1, timer 2,
 Timer 0 is used by the arduino for millis().
 The motorsheild uses timer 2, for pwm on pins 3 & 11.
 Since the standard arduino tone function uses timer 2 also, 
 we have to use a different library "NewTone" this uses timer 1 instead
 which means no pwm on pins 10 & 9. it also means it is incompatable with the 
 standard servo library.
 So we have pwm left on pins 5 & 6 to put some leds on.
 
 
 */

#include <util/delay.h>
#include <NewPing.h> // From https://code.google.com/p/arduino-new-ping/
#include <NewTone.h> // From https://code.google.com/p/arduino-new-tone/

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

// Tone configuration
const byte tone_pin = 2;
unsigned long melody_delay = 10000; // Number of milliseconds to wait before next melody
unsigned long melody_last = 0; // When the last melody played.
// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

// Led configuration
const byte led_battery = 5;

unsigned long millis_now;

void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  
  
  // Setup Channel A
  pinMode(motorA_direction, OUTPUT); // Initiates motor pin for channel A
  pinMode(motorA_brake, OUTPUT); // Initiates break pin for channel A

  // Setup Channel B
  pinMode(motorB_direction, OUTPUT); // Initiates motor pin for channel B
  pinMode(motorB_brake, OUTPUT);  // Initiates break pin for channel B
  
  // Ensure the brakes are off
  digitalWrite(motorA_brake, LOW);   // Disengage the Brake for Channel A
  digitalWrite(motorB_brake, LOW);   // Disengage the Brake for Channel B
  
  
  batteryLevel();
  
  // Turn in place constantly.
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, LOW); // Channel B backward
  analogWrite(motorA_pwm, 255); // Channel A at max speed 
  analogWrite(motorB_pwm, 255); // Channel B at max speed
  
  playMelody();
}

void loop() {
  millis_now = millis();
  
  // Check to see if it's time for another ping.
  if (millis_now - sonar_last > sonar_delay) {
    // Remeber when this ping was done, so we know when to do the next one.
    sonar_last = millis_now;
  
    unsigned int cm = sonar.ping_cm(); // Send a ping, get ping time in microseconds (uS).
    //Serial.print("Ping: ");
    //Serial.print(cm);
    //Serial.println("cm");
  }
  
  // Check to see if it's time for another melody.
  if (millis_now - melody_last > melody_delay) {
    // Remeber when this ping was done, so we know when to do the next one.
    melody_last = millis_now;
    playMelody();
    
    batteryLevel();
  }
  
}

void playMelody() 
{
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    NewTone(tone_pin, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noNewTone(tone_pin);
  }
}

void batteryLevel()
{
  // Check battery level
  long vcc = readVcc();
  byte batt = map(vcc, 2500, 5500, 0, 255);
  analogWrite(led_battery, batt);
  Serial.print("Battery: ");
  Serial.print(vcc);
  Serial.print(" : ");
  Serial.println(batt);
}

/**
 * Read the internal voltage.
 */
long readVcc() 
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  _delay_ms(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

