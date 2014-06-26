/**
 HC-SR04 Ultrasonic Range Finder.
 Two motors attached to the motor shield.
 speaker/piezo for sound
 
 PIN ALLOCATION:
 
 13 - Motor B direction
 12 - Motor A direction
 11 - (blocked by shield)
 10 - Infrared remote receiver
 9  - Battery LED 1 (cut brake connect on bottom of the shield)
 8  - Battery LED 2 (cut brake connect on bottom of the shield)
 7  - Ping trigger
 6  - Motor B pwm
 5  - Motor A pwm
 4  - Ping echo
 3  - (blocked by shield)
 2  - Sound
 1  - not connected (Serial write)
 0  - not connected (Serial read)
 
 A0 - Motor current sensor (not used)
 A1 - Motor current sensor (not used)
 A2 - Battery LED 3
 A3 - unused
 A4 - Thumbstick vertical
 A5 - Thumbstick horizontal
 
 
 ABOUT THE TIMERS:
 
 Arduino Uno has three timers; timer 0, timer 1, timer 2
 Timers can be used without their output going to the pins, the PWM pins.

  timer 0 (controls pin 5, 6)  
  timer 1 (controls pin 10, 9)
  timer 2 (controls pin 3, 11) 
  
 Timing functions we use:
 
  timer 0 - Arduino time functions; millis()
  timer 1 - Generates the tones for sound AND movement for servos, but not at the same time.
  timer 2 - Generates the pulses for driving the motors a varying speeds
  
 PWM (timer) pins we use:
  timer 0 pin 5  - Motor A to control speed.
  timer 0 pin 6  - Motor B to control speed.
  timer 1 pin 9  - Battery LED 1.  Can only be used for digitalRead() & digitalWrite()
  timer 1 pin 10 - IR receiver.    Can only be used for digitalRead() & digitalWrite()
  timer 2 pin 3  - Cannot be used as pin (blocked by shield)
  timer 2 pin 11 - Cannot be used as pin (blocked by shield)
 
 */

#include <util/delay.h>

#include "NewPingLite.h" // Adapted from https://code.google.com/p/arduino-new-ping/
#include "NewTone.h" // Adapted from https://code.google.com/p/arduino-new-tone/

#include <IRremote.h>

// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include "SimpleTimer.h"
#include <Servo.h>
#include "Sound.h"


// Define the DIO pin used for the receiver 
#define RECV_PIN 10


#define MOTOR_SPEED_MIN_A 200
#define MOTOR_SPEED_MAX_A 230 // This motor is slower on my setup.
#define MOTOR_SPEED_MIN_B 200
#define MOTOR_SPEED_MAX_B 255

// Motor attached to channel A:
const byte motorA_direction = 12;
const byte motorA_pwm = 5; // Re-wired from the shield's default of 3
const byte motorA_brake = 9;
const byte motorA_sensor = A0;
// Motor attached to channel B:
const byte motorB_direction = 13;
const byte motorB_pwm = 6; // Re-wired from the shield's default of 11
const byte motorB_brake = 8;
const byte motorB_sensor = A1;

// Ping configuration
const byte ping_trigger = 7;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
const byte ping_echo = 4;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const int ping_distance = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const byte ping_delay = 200; // Number of milliseconds to wait before next ping
NewPingLite sonar(ping_trigger, ping_echo, ping_distance); // NewPing setup of pins and maximum distance.
int ping_timer = 0; // Stores the timer used for the ping sonar.

// Whiskers setup
const byte whiskers_horiz = A5; // The analog input pin
int whiskers_horiz_default = 512; // Assume half of full analoge read.
const byte whiskers_delay = 200; // How often to check the whiskers (milliseconds).
int whiskers_timer = 0; // Stores the timer used for the whiskers.

// Battery configuration
unsigned long battery_delay = 15000; // Number of milliseconds to wait before next battery reading

// Sound configuration
const byte sound_pin = 2;
// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };
unsigned long melody_delay = 10000; // Number of milliseconds to wait before next melody

/* Structure containing received data */
decode_results results;/* Used to store the last code received. Used when a repeat code is received */
unsigned long LastCode;

/* Create an instance of the IRrecv library */
IRrecv irrecv(RECV_PIN);

// The motion states, so we always know where we're going.
enum motion_states {
  M_STOP, 
  M_FWD, 
  M_REV,
  M_ROTATE_LEFT,
  M_ROTATE_RIGHT
};
// The curent motion state.
motion_states motion_state = M_STOP;

// The action states, so we always know what action is currently happening.
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

// Create the sound playing object.
Sound snd(sound_pin);

//ServoTimer2 servo;
Servo servo_pinger;

void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to see debugging messages.
  
  irrecv.enableIRIn();
  /* Initialise the variable containing the last code received */
  LastCode = 0;
  
  pinMode(3, INPUT); // Re-wired from the shield's default
  pinMode(11, INPUT); // Re-wired from the shield's default
  
  
  // Setup Channel A
  pinMode(motorA_direction, OUTPUT); // Initiates motor pin for channel A
  pinMode(motorA_brake, OUTPUT); // Initiates break pin for channel A

  // Setup Channel B
  pinMode(motorB_direction, OUTPUT); // Initiates motor pin for channel B
  pinMode(motorB_brake, OUTPUT);  // Initiates break pin for channel B
  
  // Ensure the brakes are off
  digitalWrite(motorA_brake, LOW);   // Disengage the Brake for Channel A
  digitalWrite(motorB_brake, LOW);   // Disengage the Brake for Channel B
  
  int timer_battery = timer.setInterval(battery_delay, batteryLevel);
  Serial.print("Battery tick started id=");
  Serial.println(timer_battery);
  
  actionStop();
  batteryLevel();
  whiskersCalibrate();

}

void loop() 
{
  
    /* Has a new code been received? */
  if (irrecv.decode(&results))
  {
    /* If so get the button name for the received code */
    irHandleInput(results.value);
    //GetIRIndex(results.value);
    //Serial.println(GetIRIndex(results.value));
    /* Start receiving codes again*/
    irrecv.resume();
  }
  
  // Let the timers do their thing.
  timer.run();
  actionRun();
  snd.update();
}

void batteryLevel()
{
  // Check battery level
  long vcc = readVcc();
  byte batt = map(vcc - 3000, 0, 2000, 0, 255);
  //analogWrite(battery_led, batt);
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

void pingStart()
{
  if (ping_timer == 0) {
    ping_timer = timer.setInterval(ping_delay, pingMeasure);
  } else {
    timer.enable(ping_timer); 
  }
  Serial.println("Ping started"); 
}

void pingStop()
{
  timer.disable(ping_timer);
  Serial.println("Ping stopped"); 
}

void pingMeasure()
{
  unsigned int cm = sonar.ping_cm(); // Send a ping, get ping distance in cm.
  //unsigned int cm = 0;
  if (action_state == A_PINGSEARCH) {
    
    // looking for a new way to go.
    //int max_cm = 
    
  } else {
    
    // Watch for obstacles as we are not doing a ping search.
    if (cm > 0 && cm < 20) {
      // too close! 
      Serial.println("Too close!");
      
      // No more pinging for now.
      pingStop();
      
      // Don't hit the obstacle.
      motionStop();
      
      // Make sure it's safe to play sounds
      servoDetachAll();

      // play sound
      playMelody();


      actionPingSearch();

    }
    /*
    Serial.print("Ping: ");
    Serial.print(cm);
    Serial.println("cm");
    */
  }
}



/********************************************************************************
Action functions
********************************************************************************/
/**
 * The "state machine" that handles action.
 */
void actionRun()
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
           // reverse for 1 second
           actionTimeoutStart(1000);
           motionRev(); 
           Serial.println("SEARCH REVERSE");    
          break;
          
        case M_REV:
          if (action_done) {
            Serial.println("SEARCH LEFT");
            // Start pinging
            pingStart();
          
            actionTimeoutStart(500); //@todo not fake the search!
            motionRotateLeft(); 
          }
          break;
          
        case M_ROTATE_LEFT:
          if (action_done) {
            Serial.println("SEARCH RIGHT");
            actionTimeoutStart(2000); //@todo not fake the search!
            motionRotateRight(); 
          }
          break;
          
        case M_ROTATE_RIGHT:
          if (action_done) {
            Serial.println("SEARCH DONE");
            // Done the sweep, stop.
            actionStop(); 
            
            // Stop pinging
            pingStop(); 
            
            
            // Trundle away
            actionTrundle();
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

void actionStop()
{
  action_state = A_STOPPED;
  motionStop();
  pingStop();
  whiskersStop();
}

/**
 * Moves forward avoiding obstacles.
 */
void actionTrundle()
{
  if (action_state != A_TRUNDLE) {
    Serial.println("TRUNDLING...");
    // Remeber that we're trundling along.
    action_state = A_TRUNDLE;
    
    // Start pinging
    pingStart();
    
    // Start feeling
    whiskersStart();
  
    motionFwd();
  }
}

/**
 * Look for somewhare to go.
 */
void actionPingSearch()
{
  Serial.println("SEARCHING");
  // Set the state, so the action can be handeled by the state machine.
  action_state = A_PINGSEARCH;
}

void actionTimeoutStart(int duration)
{
  action_done = 0; 
  // Delete the previous timer, so we can use it's slot.
  timer.deleteTimer(timer_action); 
  // Start a new one.
  timer_action = timer.setTimeout(duration, actionTimeoutDone);
}

void actionTimeoutDone()
{
  action_done = 1; 
}

/********************************************************************************
Motion functions
********************************************************************************/

void motionStop()
{
  
    analogWrite(motorA_pwm, 0); // Channel A at max speed 
    analogWrite(motorB_pwm, 0); // Channel B at max speed 
    motion_state = M_STOP;
  
}

void motionFwd()
{
//@todo: set speed
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, HIGH); // Channel B forward
  analogWrite(motorA_pwm, MOTOR_SPEED_MAX_A); // Channel A at max speed 
  analogWrite(motorB_pwm, MOTOR_SPEED_MAX_B); // Channel B at max speed 
  motion_state = M_FWD;
}

void motionRev()
{

    digitalWrite(motorA_direction, LOW); // Channel A backward
    digitalWrite(motorB_direction, LOW); // Channel B backward
    analogWrite(motorA_pwm, MOTOR_SPEED_MIN_A); // Channel A at minimum speed 
    analogWrite(motorB_pwm, MOTOR_SPEED_MIN_B); // Channel B at minimum speed   
    motion_state = M_REV; 
  
}

void motionRotateLeft()
{
  
    Serial.println("LEFT");
    digitalWrite(motorA_direction, HIGH); // Channel A forward
    digitalWrite(motorB_direction, LOW); // Channel B backward
    analogWrite(motorA_pwm, MOTOR_SPEED_MIN_A);
    analogWrite(motorB_pwm, MOTOR_SPEED_MIN_B);   
    motion_state = M_ROTATE_LEFT; 
  
}

void motionRotateRight()
{
  
    Serial.println("RIGHT");
    digitalWrite(motorA_direction, LOW); // Channel A backward
    digitalWrite(motorB_direction, HIGH); // Channel B forward
    analogWrite(motorA_pwm, MOTOR_SPEED_MIN_A);
    analogWrite(motorB_pwm, MOTOR_SPEED_MIN_B);   
    motion_state = M_ROTATE_RIGHT; 
  
}

/********************************************************************************
Whiskers functions
********************************************************************************/

void whiskersCalibrate()
{
  whiskers_horiz_default = analogRead(whiskers_horiz);
  Serial.print("whisker calibration: ");
  Serial.println(whiskers_horiz_default);
}

void whiskersStart()
{
  if (whiskers_timer == 0) {
    whiskers_timer = timer.setInterval(whiskers_delay, whiskersCheck);
  } else {
    timer.enable(whiskers_timer); 
  }
  Serial.println("Whiskers started");
}

void whiskersStop()
{
  timer.disable(whiskers_timer);
  Serial.println("Whiskers stopped");
}

void whiskersCheck()
{
  int val = analogRead(whiskers_horiz);
  
  //Serial.print("whisker read: ");
  //Serial.println(val);
  
  // If the reading of the thumbstick is not the same as when calibrated,
  // we've hit something.
  if (action_state == A_PINGSEARCH) {
    
    // Doing a ping search already, so don't interfere with a whiskers bump. 
    
  } else if ( (val > whiskers_horiz_default + 30) || (val < whiskers_horiz_default - 30) ) {
    Serial.println(whiskers_horiz_default);
       Serial.println("BUMP!");
      
      // Don't hit the obstacle.
      motionStop();
      
      actionPingSearch();
  }
  
}


/********************************************************************************
IR remote functions
********************************************************************************/

/* Function returns the button name relating to the received code */
void irHandleInput(unsigned long code){
  /* Character array used to hold the received button name */
  char CodeName[3];
  /* Is the received code is a repeat code (NEC protocol) */
  if (code == 0xFFFFFFFF)
  {
    /* If so then we need to find the button name for the last button pressed */
    code = LastCode;
  }
  /* Save this code incase we get a repeat code next time */
  LastCode = code;
  
  /* Find the button name for the received code */
  switch (code)
  {
    /* Received code is for the POWER button */
  case 0xFFA25D:
    strcpy (CodeName, "PW");
    break;
    /* Received code is for the MODE button */
  case 0xFF629D:
    strcpy (CodeName, "MO");
    break;
    /* Received code is for the MUTE button */
  case 0xFFE21D:
    strcpy (CodeName, "MU");
    break;
    /* Received code is for the PLAY/PAUSE button */
  case 0xFF22DD:
    strcpy (CodeName, "RW");
    break;
    /* Received code is for the REWIND button */
  case 0xFF02FD:
    strcpy (CodeName, "FW");
    break;
    /* Received code is for the FAST FORWARD button */
  case 0xFFC23D:
    strcpy (CodeName, "PL");
    actionTrundle();
    break;
    /* Received code is for the EQ button */
  case 0xFFE01F:
    strcpy (CodeName, "EQ");
    break;
    /* Received code is for the VOLUME - button */
  case 0xFFA857:
    strcpy (CodeName, "-");
    break;
    /* Received code is for the VOLUME + button */
  case 0xFF906F:
    strcpy (CodeName, "+");
    break;
    /* Received code is for the number 0 button */
  case 0xFF6897:
    strcpy (CodeName, "0");
    break;/* Received code is for the RANDOM button */
  case 0xFF9867:
    strcpy (CodeName, "RN");
    break;
    /* Received code is for the UD/SD button */
  case 0xFFB04F:
    strcpy (CodeName, "SD");
    break;
    /* Received code is for the number 1 button */
  case 0xFF30CF:
    strcpy (CodeName, "1");
    actionStop();
    break;
    /* Received code is for the number 2 button */
  case 0xFF18E7:
    strcpy (CodeName, "2");
    break;
    /* Received code is for the number 3 button */
  case 0xFF7A85:
    strcpy (CodeName, "3");
    break;
    /* Received code is for the number 4 button */
  case 0xFF10EF:
    strcpy (CodeName, "4");
    break;
    /* Received code is for the number 5 button */
  case 0xFF38C7:
    strcpy (CodeName, "5");
    break;
    /* Received code is for the number 6 button */
  case 0xFF5AA5:
    strcpy (CodeName, "6");
    break;
    /* Received code is for the number 7 button */
  case 0xFF42BD:
    strcpy (CodeName, "7");
    break;
    /* Received code is for the number 8 button */
  case 0xFF4AB5:
    strcpy (CodeName, "8");
    break;
    /* Received code is for the number 9 button */
  case 0xFF52AD:
    strcpy (CodeName, "9");
    break;
    /* Received code is an error or is unknown */
  default:
    strcpy (CodeName, "??");
    break;
  }
  
 Serial.println(CodeName);
}

/********************************************************************************
Experimental functions
********************************************************************************/

void servoMove() 
{
  Serial.println("experimental_servoMove");
  
  
  // Can't move servo & motor at the same time.
  //action_stop();
 
  servo_pinger.attach(6);
  servo_pinger.write(random(1100, 1800));
  timer.setTimeout(500, servoDetach);
}

void servoDetach()
{
  servo_pinger.detach(); 
  //EXPERIMENTAL!!
  actionTrundle();
}

void servoDetachAll()
{
  servo_pinger.detach();
}

