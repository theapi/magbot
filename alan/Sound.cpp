/*
  Sound.cpp 
*/

#include "Arduino.h"
#include "Sound.h"

namespace sound_caller
{
    Sound *snd = 0;

    void stopNote()
    {
        if (snd)
            snd->stopNote();
    }
    
    void playNote()
    {
        if (snd)
            snd->playNote();
    }
}

Sound::Sound(int pin)
{
  _pin = pin;
  _current_note = 0;
  
  // Create our timer
  SimpleTimer _timer;
  
  // Disabled by default so as not to make the servo crazy.
  this->_enabled = 0;
  
  sound_caller::snd = this;
}

void Sound::enable()
{
  this->_enabled = 1;
}


void Sound::disable()
{
  this->_enabled = 0;
}

void Sound::update()
{
  _timer.run();
}

void Sound::stopNote()
{
  // stop the tone playing:
  noNewTone(_pin);
  // play the next one if there is one
  playNote();
}

void Sound::playNote()
{
  
  if (!this->_enabled) {
    // Not going to play sound unless explicitly told it's safe to do so.
    // ALL servos must be detached.
   return; 
  }
  
  // @todo: remove hard coding to the 8 note melody
  if (_current_note > 8) {
    // Nothing left to play
    this->disable();
    return;
  }
  
  int dur = _noteDurations[_current_note];
  int noteDuration = 1000/dur;
  this->_playing = 1;
  NewTone(_pin, _melody[_current_note], noteDuration);
  
  // to distinguish the notes, set a minimum time between them.
  // the note's duration + 30% seems to work well:
  int pauseBetweenNotes = noteDuration * 1.30;
  Serial.print(_melody[_current_note]); 
  Serial.print(" : ");
  Serial.println(pauseBetweenNotes);
  
  ++_current_note;
  _timer.setTimeout(pauseBetweenNotes, sound_caller::stopNote);

}

void Sound::playMelody(int melody[], int noteDurations[]) 
{
  this->_melody = melody;
  this->_noteDurations = noteDurations;
  this->enable();
  _current_note = 0;
  playNote();
}


