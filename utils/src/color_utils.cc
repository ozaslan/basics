#include "color_utils.hh"

namespace utils{
  namespace colors{

    Colors::Colors(){
      _init_colors();
    }

    int Colors::_init_colors(){
      if(_colors.size() == 0){
        _colors.push_back(Eigen::Vector3d(0.00, 0.00, 0.00));
        _colors.push_back(Eigen::Vector3d(1.00, 1.00, 1.00));
        _colors.push_back(Eigen::Vector3d(1.00, 0.00, 0.00));
        _colors.push_back(Eigen::Vector3d(0.00, 1.00, 0.00));
        _colors.push_back(Eigen::Vector3d(0.00, 0.00, 1.00));
        _colors.push_back(Eigen::Vector3d(1.00, 1.00, 0.00));
        _colors.push_back(Eigen::Vector3d(0.00, 1.00, 1.00));
        _colors.push_back(Eigen::Vector3d(1.00, 0.00, 1.00));
        _colors.push_back(Eigen::Vector3d(1.00, 0.60, 0.00));
        _colors.push_back(Eigen::Vector3d(0.50, 0.50, 1.00));
        _colors.push_back(Eigen::Vector3d(0.25, 0.25, 0.90));
        _colors.push_back(Eigen::Vector3d(0.00, 0.50, 0.00));
        _colors.push_back(Eigen::Vector3d(1.00, 0.50, 0.50));
        _colors.push_back(Eigen::Vector3d(0.00, 0.00, 0.80));
        _colors.push_back(Eigen::Vector3d(0.80, 0.80, 0.80));
        _colors.push_back(Eigen::Vector3d(0.50, 0.00, 0.90));
        _colors.push_back(Eigen::Vector3d(0.30, 0.80, 0.00));
        _colors.push_back(Eigen::Vector3d(1.00, 0.50, 1.00));
        _colors.push_back(Eigen::Vector3d(0.00, 0.80, 0.80));
        _colors.push_back(Eigen::Vector3d(0.90, 0.75, 0.00));
        _colors.push_back(Eigen::Vector3d(0.96, 0.00, 0.63));
        _colors.push_back(Eigen::Vector3d(0.75, 0.00, 1.00));
        _colors.push_back(Eigen::Vector3d(0.56, 0.00, 1.00));
        _colors.push_back(Eigen::Vector3d(0.50, 0.09, 0.09));
        _colors.push_back(Eigen::Vector3d(0.31, 0.47, 0.26));
        _colors.push_back(Eigen::Vector3d(254, 163, 170) / 255);
        _colors.push_back(Eigen::Vector3d(248, 184, 139) / 255);
        _colors.push_back(Eigen::Vector3d(250, 248, 132) / 255);
        _colors.push_back(Eigen::Vector3d(186, 237, 145) / 255);
        _colors.push_back(Eigen::Vector3d(178, 206, 254) / 255);
        _colors.push_back(Eigen::Vector3d(242, 162, 232) / 255);
      }
      return _colors.size();
    }

    Eigen::Vector3d Colors::operator[](int i){ 
      return _colors[i & _colors.size()];
    }

    int Colors::size(){ 
      return _colors.size();
    }
  }
}

