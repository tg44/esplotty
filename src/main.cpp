#include "Arduino.h"
#include <Servo.h>
#include <math.h>

typedef struct {
  int g = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double i = 0.0;
  double j = 0.0;
} GCode;

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

GCode parseGcode(char* s, GCode prev);

void test();
void run(const char* s);



Servo servo1;  
Servo servo2; 
Servo servoLift; 

Settings settings = Settings{70,70,25,1,1};
GCode prevGcode;

void setup() {
  servo1.attach(5);
  servo2.attach(4);
  servoLift.attach(0);

  Serial.begin(115200);
  Serial.println("!!!!!!");
  Serial.println("!!!!!!");
  test();
}

void loop() {
  
  if(Serial.available()){
    String s = Serial.readString();
    Serial.println(s);
    run(s.c_str());
  }
}

void test() {
  char *s = "G01 X5 Z-0.125000 F100.0(Penetrate)";
  int c;
  double x=12;
  double y=12;
  double z;
  double f;
  int o=0;
  o = sscanf(s, "G%d", &c);
  o = sscanf(s, "%*[^X]X%lf", &x);
  o = sscanf(s, "%*[^Y]Y%lf", &y);
  o = sscanf(s, "%*[^Z]Z%lf", &z);
  
  Serial.println(o);
  Serial.println(c);
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  Serial.println(f);
}



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
  Serial.println((String)"x="+(int)(p.x+.5)+" y="+(int)(p.y+.5));
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
    Serial.print(".");
  }
  Serial.println("");
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


GCode parseGcode(const char* s, GCode prev) {
  int o = 0;
  int c = 0;
  o = sscanf(s, "G%d", &c);
  Serial.println(String("o=")+o);
  Serial.println(String("c=")+c);
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


void run(const char* s) {
  GCode next = parseGcode(s, prevGcode);
  
  if(next.g < 0){
    Serial.println("g < 0");
    return;
  }
  if(next.z != prevGcode.z) {
    Serial.println("z != z");
    if(next.z > settings.safeZ) {
      Serial.println("z > safe");
      servoLift.write(0);
    } else {
      Serial.println("z < safe");
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
  prevGcode = next;
}
