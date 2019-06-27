#include "collision_detect.h"

#include <iostream>
#include <fstream>
#include <ctime>


namespace COLLISION_DETECTION {

namespace PMP = CGAL::Polygon_mesh_processing;

bool CollisionDetect::initialize(std::string filename1, std::string filename2) {
  std::ifstream input1(filename1);
  if (!input1 || !(input1 >> mesh1_) || !CGAL::is_triangle_mesh(mesh1_))
  {
    std::cerr << "Not a valid input file: " << filename1 << std::endl;
    return false;
  }

  std::ifstream input2(filename2);
  if (!input2 || !(input2 >> mesh2_) || !CGAL::is_triangle_mesh(mesh2_))
  {
    std::cerr << "Not a valid input file: " << filename2 << std::endl;
    return false;
  }

  // debug
  // std::cout << "mesh2_: " << std::endl;
  // for (auto vertex_iterator : mesh2_.vertices()) {
  //   K::Point_3 p = mesh2_.point(vertex_iterator);
  //   std::cout << "p: " << p << std::endl;
  // }

  // This is deep copy according to
  //  https://doc.cgal.org/latest/Surface_mesh/classCGAL_1_1Surface__mesh.html
  mesh_transformed1_ = mesh1_;
  mesh_transformed2_ = mesh2_;
  return true;
}

void CollisionDetect::setTransformation1(const Eigen::Matrix3d &R,
        const Eigen::Vector3d &p) {
  mesh_transformed1_ = mesh1_;
  K::Aff_transformation_3 transform(
      R(0,0), R(0,1), R(0,2), p(0),
      R(1,0), R(1,1), R(1,2), p(1),
      R(2,0), R(2,1), R(2,2), p(2), 1);
  PMP::transform (transform, mesh_transformed1_);
}

void CollisionDetect::setTransformation2(const Eigen::Matrix3d &R,
        const Eigen::Vector3d &p) {
  mesh_transformed2_ = mesh2_;
  K::Aff_transformation_3 transform(
      R(0,0), R(0,1), R(0,2), p(0),
      R(1,0), R(1,1), R(1,2), p(1),
      R(2,0), R(2,1), R(2,2), p(2), 1);
  // for (int i = 0; i < 4; ++i) {
  //   for (int j = 0; j < 4; ++j) {
  //     std::cout << transform.hm(i, j) << " ";
  //   }
  //   std::cout << std::endl;
  // }
  PMP::transform (transform, mesh_transformed2_);
}

bool CollisionDetect::checkCollisions(){
  return PMP::do_intersect(mesh_transformed1_, mesh_transformed2_);
}

void CollisionDetect::draw(){
  CGAL::draw(mesh_transformed1_);
}

} // namespace COLLISION_DETECTION