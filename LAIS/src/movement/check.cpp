#include "check.h"
check::check(){


}

check::check(double time, double start_x, double start_y, double start_z, double start_yaw, double obj_x, double obj_y, double obj_z)
{
    this->start_time = time;
    static GeneralMove turn(time, start_x, start_y, start_z, start_yaw, start_x, start_y, start_z, atan2(obj_y-start_y, obj_x-start_x), 6);
    this->turn = turn;
//    static GeneralMove move();

}

