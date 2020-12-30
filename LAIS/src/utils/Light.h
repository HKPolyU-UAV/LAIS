
#include "LightState.h"

class Light
{
public:
  Light();
  inline LightState * getCurrentState () const {return currentstate;}
  void toggle ();
  void setState(LightState & newState);

private:
  LightState * currentstate;






};


