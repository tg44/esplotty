

#include <Servo.h>
#include <math.h>

Servo servo1;  
Servo servo2; 
Servo servoLift; 

void setup() {
  servo1.attach(5);
  servo2.attach(4);
  servoLift.attach(0);
}

void loop() {
  int pos;

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos);              // tell servo to go to position in variable 'pos'
    servo2.write(pos);    
    servoLift.write(pos);    
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);         
    servo2.write(pos);    
    servoLift.write(pos);  
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}


typedef struct point {
  double x;
  double y;
} Point;

typedef struct settings {
  double a1;
  double a2;
  int del;
  double res;
  double safeZ;
} Settings;

Point aToB(Point a, Point b);
double length(Point v);
double distance(Point a, Point b);
Point mul(Point p, double d);
Point calcAngles(Point p, Settings s);
void moveToAngle(Point p, Settings s);

void moveFastTo(Point p, Settings s);
void moveFastRelativeTo(Point o, Point v, Settings s);
void moveStraightTo(Point o, Point p, Settings s);
void moveStraightRelativeTo(Point o, Point v, Settings s);

void interpolateLine(Point start, Point v, double steps, Settings s);
double calcAngle(Point o, Point p);
double arcLengthCCW(Point o, Point curr, Point end);
double arcLengthCW(Point o, Point curr, Point end);
Point getXYFromCycle(Point o, double r, double omega);

void interpolateCCW(Point o, Point curr, Point end, Settings s);
void interpolateCW(Point o, Point curr, Point end, Settings s);





Point aToB(Point a, Point b) {
  return Point{b.x-a.x, b.y - a.y};
}

double length(Point v) {
  return sqrt(v.x * v.x + v.y * v.y);
}

double distance(Point a, Point b) {
  Point v = aToB(a, b);
  return length(v);
}
Point mul(Point p, double d) {
  return Point{p.x*d, p.y*d};
}

// based on https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
Point calcAngles(Point p, Settings s) {
  double q2 = acos((p.x*p.x + p.y*p.y - s.a1*s.a1 - s.a2*s.a2)/(2*s.a1*s.a2));
  double q1 = atan2(p.y, p.x) - atan2(s.a2*sin(q2), s.a1+s.a2*cos(q2));
  return Point{(q1 * 4068) / 71, (q2 * 4068) / 71}; //this is rad to deg conversion
}

void moveToAngle(Point p, Settings s) {
  servo1.write((int)(p.x+.5)); //round it      
  servo2.write((int)(p.y+.5)); //round it
  delay(s.del);
}

// move to p
void moveFastTo(Point p, Settings s) {
  Point a = calcAngles(p, s);
  moveToAngle(a, s); 
}

// move from Origin with Vector
void moveFastRelativeTo(Point o, Point v, Settings s) {
  Point to = Point{o.x + v.x, o.y + v.y};
  moveFastTo(to, s);
}

// move straight from Origin to Point
void moveStraightTo(Point o, Point p, Settings s) {
  double dist = distance(o,p);
  Point v = mul(aToB(o,p), 1/(dist*(1/s.res)));
  interpolateLine(o, v, dist*(1/s.res), s);
  moveFastTo(p, s);
}

// move straight from Origin with Vector
void moveStraightRelativeTo(Point o, Point v, Settings s) {
  double dist = length(v);
  Point v2 = mul(v, 1/(dist*(1/s.res)));
  interpolateLine(o, v2, dist*(1/s.res), s);
  moveFastRelativeTo(o, v, s);
}

void interpolateLine(Point start, Point v, double steps, Settings s) {
  Point actualPos = start;
  for(int i=0; i<steps; i++) {
    Point to = Point{actualPos.x + v.x, actualPos.y + v.y};
    Point a = calcAngles(to, s);
    actualPos = to;
    moveToAngle(a, s);
  }
}

double calcAngle(Point o, Point p) {
  double dist = distance(o,p);
  if(p.y > o.y) {
    return acos((p.x-o.x)/dist);
  } else {
    return 2*M_PI - acos((p.x-o.x)/dist);
  }
}

double arcLengthCCW(Point o, Point curr, Point end) {
  double omega = calcAngle(o, end) - calcAngle(o, curr);
  if(omega < 0) omega = omega + 2*M_PI;
  return distance(o,curr) * omega;
}

double arcLengthCW(Point o, Point curr, Point end) {
  double omega = calcAngle(o, end) - calcAngle(o, curr);
  if(omega < 0) omega = omega + 2*M_PI;
  return distance(o,curr) * (2*M_PI-omega);
}

Point getXYFromCycle(Point o, double r, double omega) {
  return Point{o.x+r*cos(omega),o.y+r*sin(omega)};
}

void interpolateCCW(Point o, Point curr, Point end, Settings s) {
  double arcLength = arcLengthCCW(o, curr, end);
  double r = distance(o,curr);
  double d = arcLength / s.res;
  double smallOmega = d/r;
  double startOmega = calcAngle(o, curr);
  double currentOmega = startOmega;
  for(int i =0; i< d; i++){
    startOmega = startOmega + smallOmega;
    moveFastTo(getXYFromCycle(o, r, startOmega), s);
  }
  moveFastTo(end, s);
}

void interpolateCW(Point o, Point curr, Point end, Settings s) {
  double arcLength = arcLengthCW(o, curr, end);
  double r = distance(o,curr);
  double d = arcLength / s.res;
  double smallOmega = d/r;
  double startOmega = calcAngle(o, curr);
  double currentOmega = startOmega;
  for(int i =0; i< d; i++){
    startOmega = startOmega - smallOmega;
    moveFastTo(getXYFromCycle(o, r, startOmega), s);
  }
  moveFastTo(end, s);
}

typedef struct {
  int g = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double i = 0.0;
  double j = 0.0;
} GCode;

GCode parseGcode(char* s, GCode prev);

GCode parseGcode(char* s, GCode prev) {
  int o = 0;
  int c = 0;
  o = sscanf(s, "%*[^G]G%d", &c);
  if(o == 1) {
    GCode next = prev;
    next.g = c;
    sscanf(s, "%*[^X]X%lf", &next.x);
    sscanf(s, "%*[^Y]Y%lf", &next.y);
    sscanf(s, "%*[^Z]Z%lf", &next.z);
    sscanf(s, "%*[^I]I%lf", &next.i);
    sscanf(s, "%*[^J]J%lf", &next.j);
    return next;
  } else {
    GCode gc;
    gc.g = -1;
    return gc;
  }
}

Settings settings;
GCode prevGcode;
void run(char* s) {
  GCode next = parseGcode(s, prevGcode);
  if(next.g < 0){
    return;
  }
  if(next.z != prevGcode.z) {
    if(next.z > settings.safeZ) {
      servoLift.write(0);
    } else {
      servoLift.write(90);
    }
  }
  Point curr = Point{prevGcode.x, prevGcode.y};
  Point o = aToB(Point{prevGcode.x, prevGcode.y}, Point{next.i, next.j});
  switch(next.g) {
    case 0:
      moveFastTo(Point{next.x, next.y}, settings);
      break;
    case 1:
      moveStraightTo(curr, Point{next.x, next.y}, settings);
      break;
    case 2:
      interpolateCW(o, curr, Point{next.x, next.y}, settings);
      break;
    case 3:
      interpolateCCW(o, curr, Point{next.x, next.y}, settings);
      break;
    default:
      break;
  }
}
