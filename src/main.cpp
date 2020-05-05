#include "Arduino.h"
#include "ServoMover.h"
#include "MoverMath.h"
#include "GCodeParser.h"

ServoMover servoMover;
MoverMath moverMath;

GCode prevGcode;

void setup() {
  //random constants:
  // 5, 4, 0 the first, second and Z servo pins
  // 25 is the servo delay
  // 0 and 90 the pen up/down angles in degrees
  // 70 70 is the first and second hand distance in mm
  // 1 is the resolution (means we do a mm resolution)
  servoMover.setup(5, 4, 0, 25, 0, 90);
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
    case 66:
        servoMover.penUp();
        moverMath.moveFastRelativeTo(Point{next.x, next.y}, Point{-2, -2});
        servoMover.penDown();
        moverMath.moveStraightRelativeTo(Point{next.x, next.y}, Point{2, 2});
        servoMover.penUp();
        moverMath.moveFastRelativeTo(Point{next.x, next.y}, Point{2, -2});
        servoMover.penDown();
        moverMath.moveStraightRelativeTo(Point{next.x, next.y}, Point{-2, 2});
        servoMover.penUp();
        break;
    case 80:
        servoMover.moveToAngle(next.x, next.y);
        servoMover.lifterToAngle(next.z);
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
