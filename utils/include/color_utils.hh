#ifndef __COLOR_UTILS_HH__
#define __COLOR_UTILS_HH__

#include <Eigen/Dense> 
#include <vector> 
#include <iostream> 

using namespace std;

namespace utils{
  namespace colors{
    class Colors{
      private:
        vector<Eigen::Vector3d> _colors;
        int _init_colors();
      public:
        Colors();
        Eigen::Vector3d operator[](int i);
        int size();
    };
  }
}
#endif


