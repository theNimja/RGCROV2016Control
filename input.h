//input.h - used to map input from the DS4 to data for use in the main .ino

#ifndef input_h
#define input_h

#include "Arduino.h"

class Input {
  public:
    Input();
    int GetVertiIntensity();
    int GetHorizIntensity();
    char GetVertiDirection();
    char GetHorizDirection();

  

}





#endif
