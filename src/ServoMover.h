#ifndef SERVOMOVER
#define SERVOMOVER

#include "Arduino.h"
#include "Mover.h"
#include <Servo.h>

class ServoMover: public Mover {
  public:             
    void moveToAngle(float x, float y) {
        servo1.write((int)(x+.5)); //round it      
        servo2.write((int)(y+.5)); //round it
        #ifdef DEBUG
            Serial.println((String)"x="+(int)(x+.5)+" y="+(int)(y+.5));
        #endif
        delay(delayMs);
    }
    void setup(int a, int b, int c, int _delayMs) {
      servo1.attach(a);
      servo2.attach(b);
      servoLift.attach(c);
      delayMs = _delayMs;
    }
    void penUp() {
      servoLift.write(0);
      #ifdef DEBUG
            Serial.println((String)"pen up");
      #endif
      delay(delayMs);
    }
    void penDown() {
      servoLift.write(90);
      #ifdef DEBUG
            Serial.println((String)"pen down");
      #endif
      delay(delayMs);
    }
  private:
    Servo servo1;  
    Servo servo2; 
    Servo servoLift; 
    int delayMs = 15;
};
#endif