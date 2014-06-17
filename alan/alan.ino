/**
 HC-SR04 Ultrasonic Range Finder.
 Two motors attached to the motor shield.
 speaker/piezo for sound
 
 Arduino Uno has three timers; timer 0, timer 1, timer 2,
 Timer 0 is used by the arduino for millis().
 The motorshield uses timer 2, for pwm on pins 3 & 11.
 Since the standard arduino tone function uses timer 2 also, 
 we have to use a different library "NewTone" this uses timer 1 instead
 which means no pwm on pins 10 & 9. it also means it is incompatable with the 
 standard servo library.
 So we have pwm left on pins 5 & 6 to put some leds on.
 
 
 */

#include <util/delay.h>
#include <NewPing.h> // From https://code.google.com/p/arduino-new-ping/

// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include <Timer.h>   // From https://github.com/JChristensen/Timer/tree/v2.1


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

// Ping configuration
const byte ping_trigger = 7;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
const byte ping_echo = 4;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const int ping_distance = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const byte ping_delay = 50; // Number of milliseconds to wait before next ping
NewPing sonar(ping_trigger, ping_echo, ping_distance); // NewPing setup of pins and maximum distance.



// Battery configuration
const byte battery_led = 5;
unsigned long battery_delay = 15000; // Number of milliseconds to wait before next battery reading

// Tone configuration
const byte tone_pin = 2;
// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };
unsigned long melody_delay = 10000; // Number of milliseconds to wait before next melody

// Timer setup
Timer t;


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
  
  
  batteryLevel((void*)0);
  
  // Turn in place constantly.
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, LOW); // Channel B backward
  analogWrite(motorA_pwm, 255); // Channel A at max speed 
  analogWrite(motorB_pwm, 255); // Channel B at max speed
  
  
  
  sound_playMelody((void*)0);
  
  int pingEvent = t.every(ping_delay, doPing, (void*)0);
  Serial.print("Ping tick started id=");
  Serial.println(pingEvent);
  
  int melodyEvent = t.every(melody_delay, sound_playMelody, (void*)0);
  Serial.print("Melody tick started id=");
  Serial.println(melodyEvent);
  
  int batteryEvent = t.every(battery_delay, batteryLevel, (void*)0);
  Serial.print("Battery tick started id=");
  Serial.println(batteryEvent);
}

void loop() 
{
  
  // Let the timer do it's thing
  t.update();
}


void doPing(void *context)
{
  unsigned int cm = sonar.ping_cm(); // Send a ping, get ping time in microseconds (uS).
  if (cm > 0 && cm < 10) {
    // too close! 
    Serial.println("Too close!");
  }
  
  Serial.print("Ping: ");
  Serial.print(cm);
  Serial.println("cm");
  
}

void batteryLevel(void *context)
{
  // Check battery level
  long vcc = readVcc();
  byte batt = map(vcc - 3000, 0, 2000, 0, 255);
  analogWrite(battery_led, batt);
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

/********************************************************************************
Sound functions
********************************************************************************/

void sound_nextNote(void *context)
{
  int note = (int)context;
  if (note < 8) {
    ++note;
    sound_playNote((void*)note);
  }
}

void sound_stopNote(void *context)
{
  // stop the tone playing:
  noNewTone(tone_pin);
  sound_nextNote(context);
}

void sound_playNote(void *context)
{
  int note = (int)context;
  int noteDuration = 1000/noteDurations[note];
  NewTone(tone_pin, melody[note],noteDuration);

  // to distinguish the notes, set a minimum time between them.
  // the note's duration + 30% seems to work well:
  int pauseBetweenNotes = noteDuration * 1.30;
  Serial.println(pauseBetweenNotes);
  int noteEvent = t.after(pauseBetweenNotes, sound_stopNote, (void*)note);
}

void sound_playMelody(void *context) 
{
  sound_playNote((void*)0);
}

