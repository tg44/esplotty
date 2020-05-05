#ifndef MOVER
#define MOVER

class Mover {
  public:   
    //virtual void setup(int a, int b, int c, int _delayMs);          
    virtual void moveToAngle(const float& x, const float& y) = 0;
    virtual void penUp() = 0;
    virtual void penDown() = 0;
};
#endif
