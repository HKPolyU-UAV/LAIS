#include "Light.h"
#include "LightState.h"


class LightOff : public LightState
{

public:
  void enter(Light* light){}
  void toggle(Light* light);
  void exit(Light* light){}
  static LightState& getInstance();


private:
  LightOff(){}
  LightOff(const LightOff& other);
  LightOff operator=(const LightOff& other);

};

