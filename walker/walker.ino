/**
Walker

*/

// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include "SimpleTimer.h"

// Move servos at controlled speeds with https://github.com/netlabtoolkit/VarSpeedServo
#include <VarSpeedServo.h> 


#define FLH_CENTER   90 
#define FLF_CENTER   90 
#define FRH_CENTER   90 
#define FRF_CENTER   90

#define RLH_CENTER   90 
#define RLF_CENTER   90
#define RRH_CENTER   90 
#define RRF_CENTER   90 

#define FLH_MAX   110 
#define FLF_MAX   180 
#define FRH_MAX   130 
#define FRF_MAX   160

#define RLH_MAX   110 
#define RLF_MAX   160
#define RRH_MAX   110 
#define RRF_MAX   160 

#define FLH_MIN   70 
#define FLF_MIN   50 
#define FRH_MIN   70 
#define FRF_MIN   50

#define RLH_MIN   70 
#define RLF_MIN   50
#define RRH_MIN   70 
#define RRF_MIN   50 
 
#define FLH_PIN 4    // Front Left Hip servo  pin
#define FLF_PIN 5    // Front Left Foot servo  pin
#define FRH_PIN 6    // Front Left Foot switch pin
#define FRF_PIN 7    // Front Right Foot switch pin

#define RLH_PIN 8    // Rear Left Hip switch pin
#define RLF_PIN 9    // Rear Left Foot switch pin
#define RRH_PIN 10   // Rear Right Foot switch pin
#define RRF_PIN 11   // Rear Left Foot switch pin

VarSpeedServo servo_flh;  
VarSpeedServo servo_flf;
VarSpeedServo servo_frh;
VarSpeedServo servo_frf;

VarSpeedServo servo_rlh;  
VarSpeedServo servo_rlf;
VarSpeedServo servo_rrh;
VarSpeedServo servo_rrf;

enum motion_states {
  M_STOP, 
  M_FWD, 
  M_REV,
  M_ROTATE_LEFT,
  M_ROTATE_RIGHT
};
// The curent motion state.
motion_states motion_state = M_STOP;


int leg_timer = 0;
int leg_timer_fl = 0;
int leg_timer_fr = 0;
int leg_timer_rl = 0;
int leg_timer_rr = 0;

// There must be one global SimpleTimer object.
// More SimpleTimer objects can be created and run.
SimpleTimer timer;

void setup() 
{ 
  //Serial.begin(9600); 
  
  servo_flh.attach(FLH_PIN);
  servo_flf.attach(FLF_PIN);
  servo_frh.attach(FRH_PIN);
  servo_frf.attach(FRF_PIN);
  
  servo_rlh.attach(RLH_PIN);
  servo_rlf.attach(RLF_PIN);
  servo_rrh.attach(RRH_PIN);
  servo_rrf.attach(RRF_PIN);
  
  servo_flh.write(FLH_CENTER);
  servo_flf.write(FLF_CENTER);
  servo_frh.write(FRH_CENTER);
  servo_frf.write(FRF_CENTER);
  
  servo_rlh.write(RLH_CENTER);
  servo_rlf.write(RLF_CENTER);
  servo_rrh.write(RRH_CENTER);
  servo_rrf.write(RRF_CENTER);

  motionFwd();
}

void loop() 
{

  // Let the timers do their thing.
  timer.run();
}

void motionFwd()
{
  static byte state;

  switch (state) {
    
    case 0:
      legFwdFrontRight();
      state = 1;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
     
    case 1:
      legFwdRearLeft();
      state = 2;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
      
    case 2:
      servo_flf.write(FLF_MIN, 80);
      servo_rrf.write(FLF_MAX, 80);
      state = 3;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
      
    case 3:
      legFwdCompleteFrontRight();
      legFwdCompleteRearLeft();
      state = 4;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
      
    
      
    case 4:
      legFwdFrontLeft();
      state = 5;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
      
    case 5:
      legFwdRearRight();
      state = 6;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
      
    case 6:
      legFwdCompleteFrontLeft();
      legFwdCompleteRearRight();
      state = 0;
      leg_timer = timer.setTimeout(3000, motionFwd);
      break;
  }
  
}

/**
 * State machine for perfoming a forward step with the front right leg.
 */
void legFwdFrontRight()
{
  static byte state;
  // On the right side, FRH_MAX is as far back as the leg can be pulled.

  switch (state) {
    case 0:
      // Lift foot
      servo_frf.write(FRF_MAX, 80);
      // Call this function again later to sweep forward
      state = 1;
      leg_timer_fr = timer.setTimeout(200, legFwdFrontRight);
      break;
      
    case 1:
      // Sweep forward
      servo_frh.write(FRH_MIN, 20);
      // Call this function again later to put the foot down
      state = 2;
      leg_timer_fr = timer.setTimeout(2000, legFwdFrontRight);
      break;
      
    case 2:
      // Put foot down
      servo_frf.write(FRF_CENTER, 80);
      state = 0;
      break;
      
  }

}

void legFwdCompleteFrontRight()
{
  // Sweep back
  servo_frh.write(FRH_MAX, 20);
}

/**
 * State machine for perfoming a forward step with the rear left leg.
 */
void legFwdRearLeft()
{
  static byte state;
  // On the left side, FRH_MIN is as far back as the leg can be pulled.

  switch (state) {
    case 0:
      // Lift foot
      servo_rlf.write(RLF_MAX, 80);
      // Call this function again later to sweep forward
      state = 1;
      leg_timer_rl = timer.setTimeout(200, legFwdRearLeft);
      break;
      
    case 1:
      // Sweep forward
      servo_rlh.write(RLH_MAX, 20);
      // Call this function again later to put the foot down
      state = 2;
      leg_timer_rl = timer.setTimeout(2000, legFwdRearLeft);
      break;
      
    case 2:
      // Put foot down
      servo_rlf.write(RLF_CENTER, 80);
      state = 0;
      break;
  }

}

void legFwdCompleteRearLeft()
{
  // Sweep back
  servo_rlh.write(RLH_MIN, 20);
}

/**
 * State machine for perfoming a forward step with the rear left leg.
 */
void legFwdFrontLeft()
{
  static byte state;
  // On the left side, FRH_MIN is as far back as the leg can be pulled.

  switch (state) {
    case 0:
      // Lift foot
      servo_flf.write(FLF_MIN, 80);
      // Call this function again later to sweep forward
      state = 1;
      leg_timer_fl = timer.setTimeout(200, legFwdFrontLeft);
      break;
      
    case 1:
      // Sweep forward
      servo_flh.write(FLH_MAX, 20);
      // Call this function again later to put the foot down
      state = 2;
      leg_timer_fl = timer.setTimeout(2000, legFwdFrontLeft);
      break;
      
    case 2:
      // Put foot down
      servo_flf.write(FLF_CENTER, 80);
      state = 0;
      break;
  }

}

void legFwdCompleteFrontLeft()
{
  // Sweep back
  servo_flh.write(FLH_MIN, 20);
}

/**
 * State machine for perfoming a forward step with the rear left leg.
 */
void legFwdRearRight()
{
  static byte state;
  
  switch (state) {
    case 0:
      // Lift foot
      servo_rrf.write(RRF_MIN, 80);
      // Call this function again later to sweep forward
      state = 1;
      leg_timer_rr = timer.setTimeout(200, legFwdRearRight);
      break;
      
    case 1:
      // Sweep forward
      servo_rrh.write(RRH_MAX, 20);
      // Call this function again later to put the foot down
      state = 2;
      leg_timer_rr = timer.setTimeout(2000, legFwdRearRight);
      break;
      
    case 2:
      // Put foot down
      servo_rrf.write(RRF_CENTER, 80);
      state = 0;
      break;
  }

}

void legFwdCompleteRearRight()
{
  // Sweep back
  servo_rrh.write(RRH_MAX, 20);
}

