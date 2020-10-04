#ifndef YAMLREAD_H
#define YAMLREAD_H

#include "../../3rdPartLib/yaml-cpp-0.6.2/include/yaml-cpp/yaml.h"
#include "eigen_typedef.h"
#include "object.h"
#include <numeric>
using namespace std;

inline Mat3x3 Mat33FromYaml(string FilePath, string vName)
{
  Mat3x3 ret;
  YAML::Node config = YAML::LoadFile(FilePath);
  const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
  Eigen::Matrix<double,3,3,RowMajor> matRowMajor(vec_double.data());
  ret = matRowMajor;
  return ret;
}
inline Mat4x4 Mat44FromYaml(string FilePath, string vName)
{
  Mat4x4 ret;
  YAML::Node config = YAML::LoadFile(FilePath);
  const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
  Eigen::Matrix<double,4,4,RowMajor> matRowMajor(vec_double.data());
  ret = matRowMajor;
  return ret;
}
inline double getDoubleVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const double ret = config[vName].as<double>();
  return ret;
}
inline int getIntVariableFromYaml(string FilePath, string vName)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const int ret = config[vName].as<int>();
  return ret;
}
inline double getDoubleVariableFromYamlhierarchy(string FilePath, string vName_1, string vName_2)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const double ret = config[vName_1][vName_2].as<double>();
  return ret;
}
inline bool getBoolVariableFromYamlhierarchy(string FilePath, string vName_1, string vName_2)
{
  YAML::Node config = YAML::LoadFile(FilePath);
  const bool ret = config[vName_1][vName_2].as<bool>();
  return ret;
}
inline double average(const std::vector <double> & vec){
    if(vec.size() != 0)
    {
        double avg = accumulate(vec.begin(),vec.end(),0.0)/vec.size();
        return avg;
    }
    else {
       return 0.0;
    }
}
inline std::vector <double> approach_point(double x1, double y1, double x0, double y0, double radius)
{
    double k1, k2, a, b, c;

//    cout << "x1:  " << x1 << endl;
//    cout << "y0: " << y0 << endl;
//    cout << "y1:  " << y1 << endl;
//    cout << "x0 - x1 : " << x0-x1 << endl;
//    cout << "y0 - y1 : " << y0-y1 << endl;
    if (sqrt (pow(x1-x0,2) + pow(y1-y0,2)) < radius) // distance
    {
//        cout << "x1 , y1 is in the raidus " << endl;
//        cout << "center point: " << x0 << " ," << y0 << endl;
        vector <double> x{0, 0};
        return x;
    }
    else
    {
        if (abs(y0 - y1) < 0.00001){
           //cout << "the line is paralle to the x axis "<< endl;
           // y = constant
           if (abs(x1 - (x0 - radius)) < abs(x1 - (x0 + radius)) ){
               vector <double> x {x0 - radius, y0};
               return x;
           }
           else {
               vector <double> x {x0 + radius, y0};
               return x;
           }


        }
        else if (abs(x0 - x1) < 0.00001){
           //cout << "the line is paralle to the y axis "<< endl;
           // x = constant
           if (abs(y1 - (y0 - radius)) < abs(y1 - (y0 + radius)) ){
               vector <double> x {x0, y0 - radius};
               return x;
           }
           else {
               vector <double> x {x0, y0 + radius};
               return x;
           }

        }
        else
        {
            k1 = (y1-y0)/(x1-x0);
            k2 = (x1*y0 - y1*x0)/(x1-x0);
            a = 1+pow(k1,2);
            b = 2*k1*k2-2*k1*y0-2*x0;
            c = pow(x0,2) + pow(y0,2) + pow(k2,2) - 2*k2*y0 - pow(radius,2);
//            cout << "k1: " << k1 << endl;
//            cout << "k2: " << k2 << endl;
//            cout << "a: " << a << endl;
//            cout << "b: "<< b << endl;
//            cout << "c: " << c << endl;

//            cout << "delta: " << sqrt(pow(b,2)-4*a*c) << endl;
//            cout << "2a: " << (2*a) << endl;
        }

    }

    double sov1_x = (-b+sqrt(pow(b,2)-4*a*c))/(2*a);
    double sov1_y = k1 * sov1_x + k2;
    double sov2_x = (-b-sqrt(pow(b,2)-4*a*c))/(2*a);
    double sov2_y = k1 * sov2_x + k2;
//    cout << sov1_x << "," << sov1_y << endl;
//    cout << sov2_x << "," << sov2_y << endl;
    if ( sqrt(pow(x1-sov1_x,2)+pow(y1-sov1_y,2)) < sqrt(pow(x1-sov2_x,2)+ pow(y1-sov2_y,2)))
    {
        vector <double> x {sov1_x,sov1_y};
//        cout << "sov1 " << endl;
        return x;
    }
    else {
        vector <double> x {sov2_x,sov2_y};
//        cout << "sov2 " <<endl;
        return x;
    }

}
inline std::vector <double> cross_point(double x1, double y1, double x0, double y0, double value, string line_type)
{
  double k1, k2;
  if (abs(y0 - y1) < 0.00001)
  {
      //cout << "the line is paralle to the x axis "<< endl;
      // y = constant
    vector <double> x {value, y0};
    cout << x[0] << "," << x[1] << endl;

    return x;
  }
  else if (abs(x0 - x1) < 0.00001)
  {
      //cout << "the line is paralle to the y axis "<< endl;
      // x = constant
    vector <double> x {x0, value};
    cout << x[0] << "," << x[1] << endl;

    return x;

  }
  else
  {
      k1 = (y1-y0)/(x1-x0);
      k2 = (x1*y0 - y1*x0)/(x1-x0);
      if(line_type == "x")
      {
        double sov_x = value;
        double sov_y = k1 * sov_x + k2;
        vector <double> x {sov_x, sov_y};
        cout << x[0] << "," << x[1] << endl;
        return x;
      }
      else if (line_type == "y")
      {
        double sov_y = value;
        double sov_x = (sov_y - k2)/k1;
        vector <double> x {sov_x, sov_y};
        cout << x[0] << "," << x[1] << endl;

        return x;
      }

  }

}
inline void insertSort (std::vector <object> & objects, Vec3 original)
{

int i, j, tmp;
int n = objects.size();

 for (i=1; i<n; i++)
 {
     j=i;
     object tmp = objects[i];
     while ( j>0 && (tmp.worldframe - original).norm() < (objects[j-1].worldframe - original).norm())
     {
        objects[j]=objects[j-1];
        j--;
     }
     objects[j]=tmp;
 }
}
inline void IIRfilter (double & y, double x){
//    cout << "initial y:" << y << endl;
//    cout << "initial x:" << x << endl;

    if (y == 0.0 && !isnan(x))
    {
        y = x;
//        cout << " y == 0 after "  << y << endl;
    }
    else if (y != 0.0 ){
        y = 0.9 * y + 0.1 * x;
//        cout << " y != 0 and " << y << endl;
    }
}
inline int getIndex(vector <string> v, string K)
{
    auto it = find(v.begin(),v.end(), K);
    if(it != v.end())
    {
        int index = distance(v.begin(),it);
//        cout << index << endl;
        return index;

    }
    else
    {
        cout << "not find " << K << endl;
        return -1;
    }
}



#endif // YAML_EIGEN_H
