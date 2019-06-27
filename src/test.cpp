#include <ctime>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "collision_detect.h"

using namespace COLLISION_DETECTION;

int main(int argc, char *argv[])
{
  std::string filename1("../meshes/STEEL_HOOK.off");
  std::string filename2("../meshes/fingertip_8faces.off");

  CollisionDetect cd;
  cd.initialize(filename1, filename2);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  for (int i = 0; i < 1; ++i) {
    double begin = std::clock();
    Eigen::Vector3d p;
    p << 0, 2*i, 0;
    cd.setTransformation1(R, p);
    std::cout << "intersecting: " << cd.checkCollisions() << std::endl;
    std::cout << "Computation Time: " << (std::clock() - begin) / CLOCKS_PER_SEC << " sec" << std::endl;
  }
}
