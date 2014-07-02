/*
Sounds without delay() 
 */
 
#include <NewTone.h>
 
#include "pitches.h"

// The timer library lets us do things when we want them to happen
// without stopping everything for a delay.
#include "SimpleTimer.h"

#include "Sound.h"

const byte sound_pin = 8;

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4,4,4,4,4 };
  
  
  int notes[] = {
    NOTE_A3, NOTE_F3, NOTE_A2, 0, NOTE_A3, NOTE_F3, NOTE_A2, 0, NOTE_A3, NOTE_F3, NOTE_A2, 
  };
  
  int durations[] = {
    8, 8, 8, 12, 8, 8, 8, 12, 8, 8, 8, 
  };
  
// The global SimpleTimer object, to use instead of delays.
SimpleTimer timer;

// Create the sound playing object.
Sound snd(sound_pin);

void setup() 
{
  Serial.begin(9600);
  
  
  timer.setTimeout(500, soundMelody);
}

void loop() 
{
  timer.run();
  snd.update();
}

void soundMelody()
{
  int length = sizeof(melody) / sizeof(int);
  snd.playMelody(length, melody, noteDurations);
}

void soundDamage()
{
  int length = sizeof(notes) / sizeof(int);
  snd.playMelody(length, notes, durations);
}

