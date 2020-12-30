#include "object.h"

object::object(){

}
object::object(double X_w, double Y_w, double Z_w, bool obstacle, string name)
{
    this->name = name;
    this->worldframe[0] = X_w;
    this->worldframe[1] = Y_w;
    this->worldframe[2] = Z_w;
    this->obstacle = obstacle;
//    cout << "succeful create object: " <<  this->name << endl;
}

void object::display()
{
    std::cout << "the object is " << this->name << endl;
    std::cout << "the world coordinate is " << this->worldframe[0] << " "<< this->worldframe[1] << " " << this->worldframe[2] << endl;
    std::cout << "the object has "<< this->obstacle << " obstacle " << endl;

}
bool object::check_obstacle()
{
   if(this->obstacle == false)
     return false;
   else {
     return true;
   }
}
