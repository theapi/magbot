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
 
  timer 0 - Arduino time functions; millis() and motor speed
  timer 1 - Generates the tones for sound.
  timer 2 - Remote control IR
  
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
#include <NewTone.h>


// https://github.com/shirriff/Arduino-IRremote
#include <IRremote.h>

// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include "SimpleTimer.h"

#include "SoundPitches.h"
#include "Sound.h"



// Define the DIO pin used for the receiver 
#define RECV_PIN 10


#define MOTOR_SPEED_MIN_A 200
#define MOTOR_SPEED_MAX_A 255 
#define MOTOR_SPEED_MIN_B 200
#define MOTOR_SPEED_MAX_B 235 // This motor is slower on my setup.


// Right Motor attached to channel A:
const byte motorA_direction = 12;
const byte motorA_pwm = 5; // Re-wired from the shield's default of 3
const byte motorA_brake = 9;
const byte motorA_sensor = A0;
// Left Motor attached to channel B:
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
int whiskers_horiz_default = 512; // Assume half of full analogRead.
const byte whiskers_vert = A4; // The analog input pin
int whiskers_vert_default = 512; // Assume half of full analogRead.
const byte whiskers_delay = 100; // How often to check the whiskers (milliseconds).
const byte whiskers_threshold = 5; // How much the whiskers reading is allowed to fluctuate.
int whiskers_timer = 0; // Stores the timer used for the whiskers.

// Battery configuration
unsigned long battery_delay = 15000; // Number of milliseconds to wait before next battery reading

// Sound configuration
const byte sound_pin = 2;
// notes in the melody:
int melody_notes[] = {NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int melody_durations[] = {4, 8, 8, 4,4,4,4,4 };
  
  
int damage_notes[] = {NOTE_A3, NOTE_F3, NOTE_A2, 0, NOTE_A3, NOTE_F3, NOTE_A2, 0, NOTE_A3, NOTE_F3, NOTE_A2, };
int damage_durations[] = {8, 8, 8, 12, 8, 8, 8, 12, 8, 8, 8, };
  
int pause_notes[] = {NOTE_E6, NOTE_C6, NOTE_E6, NOTE_C6 };
int pause_durations[] = {8, 8, 8, 8, };

int block_notes[] = {NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4};
int block_durations[] = {8, 8, 8, 8, 8};

int power_notes[] = {
NOTE_D5, NOTE_F5, NOTE_A5, 
NOTE_E5, NOTE_G5, NOTE_B5, 
NOTE_F5, NOTE_A5, NOTE_C6 };
//int power_durations[] = {8, 8, 8, 8, 8, 8, 8, 8, 8};
int power_durations[] = {12, 12, 12, 12, 12, 12, 12, 12, 12};


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
  A_WHISKERSEARCH_LEFT,
  A_WHISKERSEARCH_RIGHT,
  A_DANCE1,
  A_DANCE2,
  A_BORED,
  A_HAPPY,
};
// The curent action state.
action_states action_state = A_STOPPED;
int timer_action = 0;
byte action_done = 0; // Set when the requested amount of action time is finished.

// There must be one global SimpleTimer object.
// More SimpleTimer objects can be created and run,
// although there is little point in doing so.
SimpleTimer timer;

// Create the sound playing object.
Sound snd(sound_pin);


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
  
  timer.setTimeout(500, soundPause);
  
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

/********************************************************************************
Battery functions
********************************************************************************/

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

      // play sound
      soundBlock();

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

  switch (action_state) {
      
    case A_WHISKERSEARCH_LEFT:
    case A_PINGSEARCH:
      switch (motion_state) {
        case M_STOP:
           // reverse for 1 second
           actionTimeoutStart(1000);
           motionRev(200); 
           Serial.println("REVERSE");    
          break;
          
        case M_REV:
          if (action_done) {
            // Rotate for a random amount of time.
            actionTimeoutStart(random(600, 900));
            Serial.println("ROTATE LEFT");
            motionRotateLeft(200); 
          }
          break;
          
        case M_ROTATE_LEFT:
          if (action_done) {
            Serial.println("SEARCH DONE");
            // Trundle away
            actionTrundle();
          }
          break;
          
        default:
          break;
      }
      break; // End A_PINGSEARCH / A_WHISKERSEARCH_LEFT
      
    case A_WHISKERSEARCH_RIGHT:
        switch (motion_state) {
        case M_STOP:
           // reverse for 1 second
           actionTimeoutStart(1000);
           motionRev(200); 
           Serial.println("REVERSE");    
          break;
          
        case M_REV:
          if (action_done) {
            //Serial.println(randNumber);
            // Rotate for a random amount of time.
            actionTimeoutStart(random(600, 900));
            Serial.println("ROTATE RIGHT");
            motionRotateRight(200); 
          }
          break;
          
        case M_ROTATE_RIGHT:
          if (action_done) {
            Serial.println("SEARCH DONE");
            // Trundle away
            actionTrundle();
          }
          break;

        default:
          break;
      }
      break; // End A_WHISKERSEARCH_RIGHT
      
    case A_DANCE1:
      switch (motion_state) {
        case M_STOP:
           // spin left for 3 seconds
           actionTimeoutStart(3000);
           motionRotateLeft(255);    
          break;
                    
        case M_ROTATE_LEFT:
          if (action_done) {
            // Go forward a random amount
            actionTimeoutStart(random(300,600));
            motionFwd(250);
          }
          break;
          
        case M_FWD:
          if (action_done) {
            // Go Back a random amount
            actionTimeoutStart(random(300,600));
            motionRev(250);
          }
          break;
          
        case M_REV:
          if (action_done) {
            Serial.println("DANCE1 DONE");
            // Trundle away
            actionTrundle();
          }
          break;
          
        default:
          break;
      }
      break; // End A_DANCE1
      
      
    case A_DANCE2:
      actionDance2StateMachine(motion_state);
      break; // End A_DANCE2
      
    case A_BORED:
      break;
      
    case A_HAPPY:
      actionHappyStateMachine(motion_state);
      break;
      
    default:
      break;

  }
}

void actionStop()
{
  Serial.println("STOP!");
  action_state = A_STOPPED;
  motionStop();
  actionTimeoutDone();
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
  
    motionFwd(250);
  }
}

/**
 * Do a dance
 */
void actionDance1()
{
  if (action_state != A_DANCE1) {
    actionStop();
    Serial.println("Dance 1");

    // Set the state, so the action can be handled by the state machine.
    action_state = A_DANCE1;
    
    // Start pinging
    pingStart();
      
    // Start feeling
    whiskersStart();
  }
}

/**
 * Do another dance
 */
void actionDance2()
{
  if (action_state != A_DANCE2) {
    actionStop();
    Serial.println("Dance 2");
    
    // Set the state, so the action can be handled by the state machine.
    action_state = A_DANCE2;
    
    // Start pinging
    pingStart();
      
    // Start feeling
    whiskersStart();
  }
}

void actionDance2StateMachine(byte motion_state)
{
   switch (motion_state) {
    case M_STOP:
      // Spin right for random amount of time
      actionTimeoutStart(random(2000,5000));
      motionRotateRight(255);    
      break;
               
    case M_ROTATE_RIGHT:
      if (action_done) {
        Serial.println("DANCE2 DONE");
        // Trundle away
        actionTrundle();
      }
      break;

    default:
      break;
  } 
}

/**
 * Happy
 */
void actionHappy()
{
  timer.setTimeout(2000, actionStop);
  if (action_state != A_HAPPY) {
    actionStop();
    Serial.println("Happy");
    
    // Set the state, so the action can be handled by the state machine.
    action_state = A_HAPPY;
  }
}

void actionHappyStateMachine(byte motion_state)
{
 
  switch (motion_state) {
    case M_STOP:
      // Spin left for random amount of time
      actionTimeoutStart(random(500,700));
      motionRotateLeft(255);    
      break;
      
    case M_ROTATE_LEFT:
      if (action_done) {
        // Spin right for random amount of time
        actionTimeoutStart(random(500,700));
        motionRotateRight(255); 
      }   
      break;
               
    case M_ROTATE_RIGHT:
      if (action_done) {
        // Spin left for random amount of time
        actionTimeoutStart(random(500,700));
        motionRotateLeft(255);
      }
      break;

    default:
      break;
  } 
}

/**
 * Look for somewhare to go.
 */
void actionPingSearch()
{
  Serial.println("SEARCHING");
  // Set the state, so the action can be handled by the state machine.
  action_state = A_PINGSEARCH;
}

/**
 * Look for somewhare to go.
 */
void actionWhiskerSearchLeft()
{
  Serial.println("WHISKERSEARCH LEFT");
  // Set the state, so the action can be handled by the state machine.
  action_state = A_WHISKERSEARCH_LEFT;
}

/**
 * Look for somewhare to go.
 */
void actionWhiskerSearchRight()
{
  Serial.println("WHISKERSEARCH RIGHT");
  // Set the state, so the action can be handled by the state machine.
  action_state = A_WHISKERSEARCH_RIGHT;
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

void motionFwd(byte speed)
{
  byte speed_a = motionSpeedCompensateA(speed);
  byte speed_b = motionSpeedCompensateB(speed);
  
  Serial.println("FORWARD");
  digitalWrite(motorA_direction, HIGH); // Channel A forward
  digitalWrite(motorB_direction, HIGH); // Channel B forward
  analogWrite(motorA_pwm, speed_a); // Channel A at max speed 
  analogWrite(motorB_pwm, speed_b); // Channel B at max speed 
  motion_state = M_FWD;
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
  motion_state = M_REV; 
  
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
  motion_state = M_ROTATE_LEFT; 
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
  motion_state = M_ROTATE_RIGHT; 
  
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
  
  if (action_state == A_PINGSEARCH 
    || action_state == A_WHISKERSEARCH_LEFT 
    || action_state == A_WHISKERSEARCH_RIGHT) {
    // Doing a search already, so don't interfere with a whiskers bump.
    return;
  }
  
  
  // If the reading of the thumbstick is not the same as when calibrated,
    // we've hit something.
  
  // Vertical check
  int val = analogRead(whiskers_vert);
  if (val > whiskers_horiz_default + whiskers_threshold || val < whiskers_vert_default - whiskers_threshold) {
    Serial.println(whiskers_vert_default);
    Serial.println("BUMP VERTICLE!");
    // Don't hit the obstacle.
    motionStop();
    soundDamage();
      
    actionWhiskerSearchRight();
    
  } else {
  
    // Horizontal check
    int val = analogRead(whiskers_horiz);
    
    //Serial.print("whisker read: ");
    //Serial.println(val);
    
    
    if (val > whiskers_horiz_default + whiskers_threshold) {
      Serial.println(whiskers_horiz_default);
      Serial.println("BUMP LEFT!");
      // Don't hit the obstacle.
      motionStop();
      soundDamage();
        
      actionWhiskerSearchRight();
    
    } else if (val < whiskers_horiz_default - whiskers_threshold) {
      Serial.println(whiskers_horiz_default);
      Serial.println("BUMP RIGHT!");
      // Don't hit the obstacle.
      motionStop();
      soundDamage();
        
      actionWhiskerSearchLeft();
    } 
  }

}

/********************************************************************************
Sound functions
********************************************************************************/

void soundMelody()
{
  Serial.println("Sound: melody");
  int length = sizeof(melody_notes) / sizeof(int);
  snd.playNotes(length, melody_notes, melody_durations);
}

void soundDamage()
{
  Serial.println("Sound: damage");
  int length = sizeof(damage_notes) / sizeof(int);
  snd.playNotes(length, damage_notes, damage_durations);
}

void soundPause()
{
  Serial.println("Sound: pause");
  int length = sizeof(pause_notes) / sizeof(int);
  snd.playNotes(length, pause_notes, pause_durations);
}

void soundBlock()
{
  Serial.println("Sound: block");
  int length = sizeof(block_notes) / sizeof(int);
  snd.playNotes(length, block_notes, block_durations);
}

void soundPower()
{
  Serial.println("Sound: power");
  int length = sizeof(power_notes) / sizeof(int);
  snd.playNotes(length, power_notes, power_durations);
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
    /* Received code is for the REWIND button */
  case 0xFF22DD:
    strcpy (CodeName, "RW");
    break;
    /* Received code is for the FAST FORWARD button */
  case 0xFF02FD:
    strcpy (CodeName, "FW");
    break;
    /* Received code is for the  PLAY/PAUSE button */
  case 0xFFC23D:
    strcpy (CodeName, "PL");
    soundPower();
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
    soundPause();
    actionStop();
    break;
    /* Received code is for the number 2 button */
  case 0xFF18E7:
    strcpy (CodeName, "2");
    actionDance1();
    break;
    /* Received code is for the number 3 button */
  case 0xFF7A85:
    strcpy (CodeName, "3");
    actionDance2();
    break;
    /* Received code is for the number 4 button */
  case 0xFF10EF:
    strcpy (CodeName, "4");
    actionHappy();
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


