/*
  Sound.h 
*/
#ifndef Sound_h
#define Sound_h

#include "Arduino.h"
#include <NewTone.h> 
#include "SimpleTimer.h"

class Sound
{
  public:
    Sound(int pin);
    void nextNote();
    void stopNote();
    void playNote(int note);
    void playMelody(int melody[], int noteDurations[]);
    void update();
  private:
    SimpleTimer _timer;
    int _pin;
    int _current_note;
    int *_melody;
    int *_noteDurations;
};



#endif

