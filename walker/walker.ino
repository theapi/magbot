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
#define FLF_MAX   110 
#define FRH_MAX   130 
#define FRF_MAX   160

#define RLH_MAX   110 
#define RLF_MAX   110
#define RRH_MAX   110 
#define RRF_MAX   110 

#define FLH_MIN   60 
#define FLF_MIN   60 
#define FRH_MIN   50 
#define FRF_MIN   60

#define RLH_MIN   60 
#define RLF_MIN   60
#define RRH_MIN   60 
#define RRF_MIN   60 
 
#define FLH_PIN 4    // Front Left Hip servo  pin
#define FLF_PIN 5    // Front Left Foot servo  pin
#define FRH_PIN 6    // Front Right Foot switch pin
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
int leg_timer_fr = 0;


// There must be one global SimpleTimer object.
// More SimpleTimer objects can be created and run.
SimpleTimer timer;

void setup() 
{ 
  Serial.begin(9600); 
  
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
  
  leg_timer = timer.setInterval(6000, legFwdFrontRight);

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
      state = 3;
      // Call this function again later to sweep back
      leg_timer_fr = timer.setTimeout(1000, legFwdFrontRight);
      break;
      
    case 3:
      // Sweep back
      servo_frh.write(FRH_MAX, 20);
      state = 0;
      break;
  }

}


