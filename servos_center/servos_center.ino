// Center 8 servos

#include <Servo.h> 
 

#define FLH_PIN 4    // Front Left Hip servo  pin
#define FLF_PIN 5    // Front Left Foot servo  pin
#define FRH_PIN 6    // Front Left Foot switch pin
#define FRF_PIN 7    // Front Right Foot switch pin

#define RLH_PIN 8    // Rear Left Hip switch pin
#define RLF_PIN 9    // Rear Left Foot switch pin
#define RRH_PIN 10   // Rear Right Foot switch pin
#define RRF_PIN 11   // Rear Left Foot switch pin


Servo servo_flh;  
Servo servo_flf;
Servo servo_frh;
Servo servo_frf;

Servo servo_rlh;  
Servo servo_rlf;
Servo servo_rrh;
Servo servo_rrf;

 
void setup() 
{ 
  servo_flh.attach(FLH_PIN);
  servo_flf.attach(FLF_PIN);
  servo_frh.attach(FRH_PIN);
  servo_frf.attach(FRF_PIN);
  
  servo_rlh.attach(RLH_PIN);
  servo_rlf.attach(RLF_PIN);
  servo_rrh.attach(RRH_PIN);
  servo_rrf.attach(RRF_PIN);
  

  servo_flh.write(90);
  servo_flf.write(90);

  servo_frh.write(90);
  servo_frf.write(90);

  servo_rlh.write(90);
  servo_rlf.write(90);

  servo_rrh.write(90);
  servo_rrf.write(90);
} 
 
 
void loop() 
{

}

