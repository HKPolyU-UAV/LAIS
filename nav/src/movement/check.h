#ifndef CHECK_H
#define CHECK_H
#include "circle.h"
#include "generalmove.h"
class check
{

public:
    check();
    check(double time, double start_x, double start_y, double start_z, double start_yaw, double obj_x, double obj_y, double obj_z);
private:
    GeneralMove turn;
    GeneralMove move;
    circleTrj inspect;
    double start_time;
    double start_x , start_y, start_z;
    double start_yaw;
    double obj_x, obj_y, obj_z;

};
#endif // CHECK_H
