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
}

Sound::Sound(int pin)
{
  _pin = pin;
  _current_note = 0;
  
  // Create our timer
  SimpleTimer _timer;
  
  sound_caller::snd = this;
}

void Sound::update()
{
  _timer.run();
}

void Sound::nextNote()
{
  ++_current_note;
  // @todo: remove hard coding to the 8 note melody
  if (_current_note < 8) {
    playNote(_current_note);
  } else {
    // reset the counter
   _current_note = 0; 
  }
}

void Sound::stopNote()
{
  // stop the tone playing:
  noNewTone(_pin);
  nextNote();
}

void Sound::playNote(int note)
{
  int dur = _noteDurations[note];
  int noteDuration = 1000/dur;
  NewTone(_pin, _melody[note], noteDuration);

  // to distinguish the notes, set a minimum time between them.
  // the note's duration + 30% seems to work well:
  int pauseBetweenNotes = noteDuration * 1.30;
  Serial.print(_melody[note]); 
  Serial.print(" : ");
  Serial.println(pauseBetweenNotes);
  _timer.setTimeout(pauseBetweenNotes, sound_caller::stopNote);
}

void Sound::playMelody(int melody[], int noteDurations[]) 
{
  this->_melody = melody;
  this->_noteDurations = noteDurations;
  playNote(_current_note);
}


