#include "Arduino.h"
#include "ServoMover.h"
#include "MoverMath.h"
#include "GCodeParser.h"

ServoMover servoMover;
MoverMath moverMath;

GCode prevGcode;

void setup() {
  servoMover.setup(5,4,0,25);
  moverMath.setup(70, 70, 1, &servoMover);

  Serial.begin(115200);
}

void runOnce(const char* s) {
  GCode next = parseGcode(s, prevGcode);
  
  if(next.g < 0){
    #ifdef DEBUG
      Serial.println("g < 0");
    #endif
    return;
  }
  if(next.z != prevGcode.z) {
    #ifdef DEBUG
      Serial.println("z != z");
    #endif
    if(next.z > 1) {
      servoMover.penUp();
    } else {
      servoMover.penDown();
    }
  }
  Point curr = Point{prevGcode.x, prevGcode.y};
  Point o = Point::aToB(Point{prevGcode.x, prevGcode.y}, Point{next.i, next.j});
  switch(next.g) {
    case 0:
      moverMath.moveFastTo(Point{next.x, next.y});
      break;
    case 1:
      moverMath.moveStraightTo(curr, Point{next.x, next.y});
      break;
    case 2:
      moverMath.moveCW(o, curr, Point{next.x, next.y});
      break;
    case 3:
      moverMath.moveCCW(o, curr, Point{next.x, next.y});
      break;
    default:
      break;
  }
  prevGcode = next;
}

void loop() {
  if(Serial.available()){
    String s = Serial.readString();
    #ifdef DEBUG
      Serial.println(s);
    #endif
    runOnce(s.c_str());
  }
}
