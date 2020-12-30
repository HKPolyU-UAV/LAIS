#include "Light.h"

class Light;

class LightState
{
public:
  virtual void enter(Light* light) = 0;  // pure void function
  virtual void toggle(Light* light) = 0;
  virtual void exit(Light* light) = 0;
  virtual ~LightState(){}
};


