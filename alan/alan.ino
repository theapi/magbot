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
#include <NewTone.h> // From https://code.google.com/p/arduino-new-tone/

// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include "SimpleTimer.h"


#include "Sound.h"

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
const byte ping_delay = 200; // Number of milliseconds to wait before next ping
NewPing sonar(ping_trigger, ping_echo, ping_distance); // NewPing setup of pins and maximum distance.
int ping_timer = 0; // Stores the timer used for the ping sonar.

// Battery configuration
const byte battery_led = 5;
unsigned long battery_delay = 15000; // Number of milliseconds to wait before next battery reading

// Sound configuration
const byte sound_pin = 2;
// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };
unsigned long melody_delay = 10000; // Number of milliseconds to wait before next melody

// The motion states, so we alwasys no where we're going.
enum motion_states {
  M_STOP, 
  M_FWD, 
  M_REV,
  M_ROTATE_LEFT,
  M_ROTATE_RIGHT
};
// The curent motion state.
motion_states motion_state = M_STOP;

// The action states, so we alwasys no what action is currently happening.
enum action_states {
  A_STOPPED,
  A_TRUNDLE, 
  A_PINGSEARCH,
  A_DANCE1,
  A_DANCE2,
  A_BORED,
  A_HAPPY,
  A_CALIBRATION
};
// The curent action state.
action_states action_state = A_STOPPED;
int timer_action = 0;
byte action_done = 0; // mnmm needs explaining

// There must be one global SimpleTimer object.
// More SimpleTimer objects can be created and run,
// although there is little point in doing so.
SimpleTimer timer;

// Creat the sound playing object.
Sound snd(sound_pin);

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
  
  // Start moving!
  timer_action = timer.setTimeout(100, action_trundle);
  //action_trundle();
  
  // Start singing
  playMelody();
  

  
  /*
  int timer_melody = timer.setInterval(melody_delay, playMelody);
  Serial.print("Melody tick started id=");
  Serial.println(timer_melody);
  */
  
  int timer_battery = timer.setInterval(battery_delay, batteryLevel);
  Serial.print("Battery tick started id=");
  Serial.println(timer_battery);
}

void loop() 
{
  // Let the timers do their thing.
  timer.run();
  action_run();
  snd.update();
}

void batteryLevel()
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

void playMelody() 
{
  snd.playMelody(melody, noteDurations);
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
Ping functions
********************************************************************************/

void ping_start()
{
  if (ping_timer == 0) {
    ping_timer = timer.setInterval(ping_delay, ping_measure);
  } else {
    timer.enable(ping_timer); 
  }
  Serial.println("Ping started"); 
}

void ping_stop()
{
  timer.disable(ping_timer);
  Serial.println("Ping stopped"); 
}

void ping_measure()
{
  unsigned int cm = sonar.ping_cm(); // Send a ping, get ping distance in cm.
  
  if (action_state == A_PINGSEARCH) {
    
    // looking for a new way to go.
    //int max_cm = 
    
  } else {
    
    // Watch for obstacles as we are not doing a ping search.
    if (cm > 0 && cm < 10) {
      // too close! 
      Serial.println("Too close!");
      
      // No more pinging for now.
      ping_stop();
      
      // Don't hit the obstacle.
      motion_stop();
      
      // play sound
      playMelody();
    
      // back up
      motion_rev();
      
      // then after a second, search for a new direction.
      timer.setTimeout(1000, action_pingSearch);
    }
    
    Serial.print("Ping: ");
    Serial.print(cm);
    Serial.println("cm");
  }
}



/********************************************************************************
Action functions
********************************************************************************/
/**
 * The "state machine" that handles action.
 */
void action_run()
{
  /*
  Serial.print("action_run ");
  Serial.print(action_state);
  Serial.print(" - ");
  Serial.println(motion_state);
  */
  
  switch (action_state) {
    case A_STOPPED:
    
      break;
    
    case A_TRUNDLE:
    
      break;
      
    case A_PINGSEARCH:
      switch (motion_state) {
        case M_STOP:
        Serial.println("SEARCH LEFT");
          // Start pinging
          ping_start();
  
          action_timeoutStart(1000);
          // Rotate left for 1 second
          motion_rotateLeft();          
          break;
          
        case M_ROTATE_LEFT:
          if (action_done) {
            Serial.println("SEARCH RIGHT");
            action_timeoutStart(1500); //@todo not fake the search!
            // Rotate left
            motion_rotateRight(); 
          }
          break;
          
        case M_ROTATE_RIGHT:
          if (action_done) {
            Serial.println("SEARCH DONE");
            // Done the sweep, stop.
            action_stop(); 
            
            // Stop pinging
            ping_stop(); 
            
            // Trundle away
            action_trundle();
          }
          break;
      }
      break;
      
    case A_DANCE1:
    
      break;
      
    case A_DANCE2:
    
      break;
      
    case A_BORED:
    
      break;
      
    case A_HAPPY:
    
      break;
      
    case A_CALIBRATION:
    
      break;
  }
}

void action_stop()
{
  action_state = A_STOPPED;
  motion_stop();
}

/**
 * Moves forward avoiding obstacles.
 */
void action_trundle()
{
  Serial.println("TRUNDLING...");
  // Remeber that we're trundling along.
  action_state = A_TRUNDLE;
  
  // Start pinging
  ping_start();

  motion_fwd();
}

/**
 * Look for somewhare to go.
 */
void action_pingSearch()
{
  Serial.println("SEARCHING");

  motion_stop();
  // Set the state, so the action can complete.
  action_state = A_PINGSEARCH;
}

void action_timeoutStart(int duration)
{
  action_done = 0; 
  // Delete the previous timer, so we can use it's slot.
  timer.deleteTimer(timer_action); 
  // Start a new one.
  timer_action = timer.setTimeout(duration, action_timeoutDone);
}

void action_timeoutDone()
{
  action_done = 1; 
}

/********************************************************************************
Motion functions
********************************************************************************/

void motion_stop()
{
  if (motion_state != M_STOP) {
    analogWrite(motorA_pwm, 0); // Channel A at max speed 
    analogWrite(motorB_pwm, 0); // Channel B at max speed 
    motion_state = M_STOP;
  }
}

void motion_fwd()
{
//@todo: set speed
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, HIGH); // Channel B forward
  analogWrite(motorA_pwm, 255); // Channel A at max speed 
  analogWrite(motorB_pwm, 255); // Channel B at max speed 
  motion_state = M_FWD;
}

void motion_rev()
{
  if (motion_state != M_REV) {
//@todo: set speed
    digitalWrite(motorA_direction, LOW); // Channel A backward
    digitalWrite(motorB_direction, LOW); // Channel B backward
    analogWrite(motorA_pwm, 125); // Channel A at half speed 
    analogWrite(motorB_pwm, 125); // Channel B at half speed   
    motion_state = M_REV; 
  }
}

void motion_rotateLeft()
{
  if (motion_state != M_ROTATE_LEFT) {
    Serial.println("LEFT");
//@todo: set speed
    digitalWrite(motorA_direction, HIGH); // Channel A forward
    digitalWrite(motorB_direction, LOW); // Channel B backward
    analogWrite(motorA_pwm, 75);
    analogWrite(motorB_pwm, 75);   
    motion_state = M_ROTATE_LEFT; 
  }
}

void motion_rotateRight()
{
  if (motion_state != M_ROTATE_RIGHT) {
    Serial.println("RIGHT");
//@todo: set speed
    digitalWrite(motorA_direction, LOW); // Channel A backward
    digitalWrite(motorB_direction, HIGH); // Channel B forward
    analogWrite(motorA_pwm, 75);
    analogWrite(motorB_pwm, 75);   
    motion_state = M_ROTATE_RIGHT; 
  }
}

