#ifndef MOVERMATH
#define MOVERMATH

#include <math.h>
#include "Mover.h"

class Point {
  public: 
    float x = 0.0f;
    float y = 0.0f;
    Point(float x = 0.0f, float y = 0.0f): x(x), y(y){};
    float length() const {
      return sqrt(x * x + y * y);
    }
    Point mul(float d) const {
      return Point{x*d, y*d};
    }
    static Point aToB(Point a, Point b) {
      return Point{b.x-a.x, b.y - a.y};
    }
    static float distance(Point a, Point b) {
      Point v = aToB(a, b);
      return v.length();
    }
};



class MoverMath {
  public:
    void setup(float _a1, float _a2, float _res, Mover* _mover) {
      a1 = _a1;
      a2 = _a2;
      res = _res;
      mover = _mover;
    }
    // move to p
    void moveFastTo(const Point& p) {
      Point a = calcAngles(p);
      mover->moveToAngle(a.x, a.y); 
    }

    // move from Origin with Vector
    void moveFastRelativeTo(const Point& o, const Point& v) {
      Point to = Point{o.x + v.x, o.y + v.y};
      moveFastTo(to);
    }

    // move straight from Origin to Point
    void moveStraightTo(const Point& o, const Point& p) {
      float dist = Point::distance(o,p);
      Point v = Point::aToB(o,p).mul(1/(dist*(1/res)));
      interpolateLine(o, v, dist*(1/res));
      moveFastTo(p);
    }

    // move straight from Origin with Vector
    void moveStraightRelativeTo(const Point& o, const Point& v) {
      float dist = v.length();
      Point v2 = v.mul(1/(dist*(1/res)));
      interpolateLine(o, v2, dist*(1/res));
      moveFastRelativeTo(o, v);
    }

    void moveCCW(const Point& o, const Point& curr, const Point& end) {
      float arcLength = arcLengthCCW(o, curr, end);
      float r = Point::distance(o,curr);
      float d = arcLength / res;
      float smallOmega = d/r;
      float currentOmega = calcAngle(o, curr);
      for(int i =0; i< d; i++){
        currentOmega = currentOmega + smallOmega;
        moveFastTo(getXYFromCycle(o, r, currentOmega));
      }
      moveFastTo(end);
    }

    void moveCW(const Point& o, const Point& curr, const Point& end) {
      float arcLength = arcLengthCW(o, curr, end);
      float r = Point::distance(o,curr);
      float d = arcLength / res;
      float smallOmega = d/r;
      float currentOmega = calcAngle(o, curr);
      for(int i =0; i< d; i++){
        currentOmega = currentOmega - smallOmega;
        moveFastTo(getXYFromCycle(o, r, currentOmega));
      }
      moveFastTo(end);
    }

  private:
    Mover* mover;
    float a1;
    float a2;
    float res;

    // based on https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
    Point calcAngles(const Point& p) {
      float q2 = acosf((p.x*p.x + p.y*p.y - a1*a1 - a2*a2)/(2*a1*a2));
      float q1 = atan2f(p.y, p.x) - atan2f(a2*sinf(q2), a1+a2*cosf(q2));
      return Point{(q1 * 4068) / 71, (q2 * 4068) / 71}; //this is rad to deg conversion
    }
    void interpolateLine(const Point& start, const Point& v, double steps) {
      Point actualPos = start;
      for(int i=0; i<steps; i++) {
        Point to = Point{actualPos.x + v.x, actualPos.y + v.y};
        actualPos = to;
        moveFastTo(to);
      }
    }
    double calcAngle(const Point& o, const Point& p) {
      double dist = Point::distance(o,p);
      if(p.y > o.y) {
        return acos((p.x-o.x)/dist);
      } else {
        return 2*M_PI - acos((p.x-o.x)/dist);
      }
    }

    float arcLengthCCW(const Point& o, const Point& curr, const Point& end) {
      float omega = calcAngle(o, end) - calcAngle(o, curr);
      if(omega < 0) omega = omega + 2*M_PI;
      return Point::distance(o,curr) * omega;
    }

    float arcLengthCW(const Point& o, const Point& curr, const Point& end) {
      float omega = calcAngle(o, end) - calcAngle(o, curr);
      if(omega < 0) omega = omega + 2*M_PI;
      return Point::distance(o,curr) * (2*M_PI-omega);
    }

    Point getXYFromCycle(const Point& o, float r, float omega) {
      return Point{o.x+r*cosf(omega),o.y+r*sinf(omega)};
    }
};

#endif