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
  std::string filename("../meshes/BIG_SCREW.off");

  RayTracing rt;
  rt.initialize(filename);

  std::vector<Eigen::Vector3d> intersections;
  double begin = std::clock();
  for (int i = 0; i < 500; ++i) {
    Eigen::Vector3d n, p;
    p << 0, 0, 25;

    n(0) = double(std::rand())/double(RAND_MAX) - 0.5;
    n(1) = double(std::rand())/double(RAND_MAX) - 0.5;
    n(2) = double(std::rand())/double(RAND_MAX) - 0.5;
    n.normalize();

    intersections = rt.findIntersections(p, n);
    if (intersections.size() > 0) {
      for (int i = 0; i < intersections.size(); ++i) {
        std::cout << intersections[i][0] << ", " << intersections[i][1] << ", " << intersections[i][2] << " | ";
      }
      std::cout << std::endl;
    }
    std::cout << "test No." << i << ", intersections: " << intersections.size() << std::endl;
  }
  std::cout << "Computation Time: " << (std::clock() - begin) / CLOCKS_PER_SEC << " sec" << std::endl;
}
