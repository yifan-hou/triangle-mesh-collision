#include <ctime>
#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "collision_detect.h"

using namespace COLLISION_DETECTION;

int main(int argc, char *argv[])
{
  std::string filename1("../meshes/BIG_SCREW.off");
  std::string filename2("../meshes/BIG_SCREW.off");

  CollisionDetect cd;
  cd.initialize(filename1, filename2);
  double begin = std::clock();
  for (int i = 0; i < 5000; ++i) {
    Eigen::Vector3d p;
    p(0) = double(std::rand())/double(RAND_MAX) - 0.5;
    p(1) = double(std::rand())/double(RAND_MAX) - 0.5;
    p(2) = double(std::rand())/double(RAND_MAX) - 0.5;
    p = 100.0*p;
    cd.setTransformation1(Eigen::Quaterniond::UnitRandom().toRotationMatrix(), p);
    std::cout << "test No." << i << ", intersecting: " << cd.checkCollisions() << std::endl;
  }
  std::cout << "Computation Time: " << (std::clock() - begin) / CLOCKS_PER_SEC << " sec" << std::endl;
}
