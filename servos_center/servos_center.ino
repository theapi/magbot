// Center 8 servos

#include <Servo.h> 
 
#define FLH_CENTER      1500 
#define FLF_CENTER      1500 
#define FRH_CENTER      1500 
#define FRF_CENTER      1500

#define RLH_CENTER      1500 
#define RLF_CENTER      1500
#define RRH_CENTER      1500 
#define RRF_CENTER      1500 
 
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
  
  servo_flh.writeMicroseconds(FLH_CENTER);
  servo_flf.writeMicroseconds(FLF_CENTER);
  servo_frh.writeMicroseconds(FRH_CENTER);
  servo_frf.writeMicroseconds(FRF_CENTER);
  
  servo_rlh.writeMicroseconds(RLH_CENTER);
  servo_rlf.writeMicroseconds(RLF_CENTER);
  servo_rrh.writeMicroseconds(RRH_CENTER);
  servo_rrf.writeMicroseconds(RRF_CENTER);
} 
 
 
void loop() 
{

}

