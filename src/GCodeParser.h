#ifndef GCODEPARSER
#define GCODEPARSER

#include "Arduino.h"

typedef struct {
  int g = 0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float i = 0.0f;
  float j = 0.0f;
} GCode;

GCode parseGcode(const char* s, const GCode &prev) {
  int o = 0;
  int c = 0;
  o = sscanf(s, "G%d", &c);
  #ifdef DEBUG
    Serial.println(String("o=")+o);
    Serial.println(String("c=")+c);
  #endif
  GCode next;
  if(o == 1) {
    next = prev;
    next.g = c;
    sscanf(s, "%*[^X]X%f", &next.x);
    sscanf(s, "%*[^Y]Y%f", &next.y);
    sscanf(s, "%*[^Z]Z%f", &next.z);
    sscanf(s, "%*[^I]I%f", &next.i);
    sscanf(s, "%*[^J]J%f", &next.j);
  } else {
    next.g = -1;
  }
  return next;
}

#endif