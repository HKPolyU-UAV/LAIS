#ifndef OBJECT_H
#define OBJECT_H
#include <string>
#include "../../3rdPartLib/yaml-cpp-0.6.2/include/yaml-cpp/yaml.h"
#include "eigen_typedef.h"
#include <numeric>
#include <iostream>
using namespace std;

class object
{
private:


public:
    Vec3 worldframe = {0,0,0};
    bool obstacle = false;
    string name;
    object();
    object(double X_w, double Y_w, double Z_w, bool obstacle, string name);
    void display();
    bool check_obstacle();

};

#endif

