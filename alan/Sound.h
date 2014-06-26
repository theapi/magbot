/*
  Sound.h 
*/
#ifndef Sound_h
#define Sound_h

#include "Arduino.h"
#include "NewToneB.h"
#include "SimpleTimer.h"

class Sound
{
  public:
    Sound(int pin);
    //void nextNote();
    void stopNote();
    void playNote();
    void playMelody(int melody[], int noteDurations[]);
    void update();
    void enable();
    void disable();
  private:
    SimpleTimer _timer;
    int _pin;
    int _current_note;
    int *_melody;
    int *_noteDurations;
    int8_t _playing; // So we know if something is playing
    int8_t _enabled;
};



#endif

