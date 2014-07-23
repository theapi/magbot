/*
Sounds without delay() 
 */
 
//#include <NewTone.h>
 


// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include "SimpleTimer.h"

#include "SoundPitches.h"
#include "Sound.h"

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
  
// The global SimpleTimer object, to use instead of delays.
SimpleTimer timer;

// Create the sound playing object.
Sound snd(sound_pin);

void setup() 
{
  Serial.begin(9600);
  
  
  timer.setTimeout(250, soundDemo);
}

void loop() 
{
  timer.run();
  snd.update();
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    
    switch(inChar) {
      case 'a':
        soundPause();
        break;
      case 'b':
        soundBlock();
        break;
      case 'c':
        soundPower();
        break;
      case 'd':
        soundDamage();
        break;
    }
    
  }
}

void soundDemo()
{
  soundPower();
  timer.setTimeout(2000, soundPause);
  timer.setTimeout(4000, soundBlock);
  timer.setTimeout(6000, soundDamage);
  //timer.setTimeout(8000, soundMelody);
}

void soundMelody()
{
  int length = sizeof(melody_notes) / sizeof(int);
  snd.playNotes(length, melody_notes, melody_durations);
}

void soundDamage()
{
  int length = sizeof(damage_notes) / sizeof(int);
  snd.playNotes(length, damage_notes, damage_durations);
}

void soundPause()
{
  int length = sizeof(pause_notes) / sizeof(int);
  snd.playNotes(length, pause_notes, pause_durations);
}

void soundBlock()
{
  int length = sizeof(block_notes) / sizeof(int);
  snd.playNotes(length, block_notes, block_durations);
}

void soundPower()
{
  int length = sizeof(power_notes) / sizeof(int);
  snd.playNotes(length, power_notes, power_durations);
}


