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
  PMP::transform (transform, mesh_transformed2_);
}

bool CollisionDetect::checkCollisions(){
  return PMP::do_intersect(mesh_transformed1_, mesh_transformed2_);
}

void CollisionDetect::draw(){
  CGAL::draw(mesh_transformed1_);
}



bool RayTracing::initialize(std::string filename) {
  std::ifstream input(filename);
  if (!input || !(input >> mesh_) || !CGAL::is_triangle_mesh(mesh_))
  {
    std::cerr << "Not a valid input file: " << filename << std::endl;
    return false;
  }
  return true;
}

std::vector<Eigen::Vector3d> RayTracing::findIntersections(
    const Eigen::Vector3d &p, const Eigen::Vector3d &n) {

  // construct the ray object
  assert(n.norm() > 1e-1); // n should not be zero length.
  Point p_cgal = Point(p[0], p[1], p[2]);
  Vector v_cgal = Vector(n[0], n[1], n[2]);
  Ray ray(p_cgal, v_cgal);
  // construct the AABB tree TODO(yifan) move it to initialization
  Tree tree(faces(mesh_).first, faces(mesh_).second, mesh_);
  // compute intersections
  std::list<Ray_intersection> intersections;

  // Ray_intersection ray_intersection = tree.first_intersection(ray);
  // if(tree.do_intersect(ray))
  //   std::cout << "intersection(s)" << std::endl;
  // else
  //   std::cout << "no intersection" << std::endl;
  // std::cout << tree.number_of_intersected_primitives(ray)
  //     << " intersection(s)" << std::endl;

  tree.all_intersections(ray, std::back_inserter(intersections));
  // read results
  std::vector<Eigen::Vector3d> intersections_eigen_format;
  for (Ray_intersection i:intersections) {
    if(boost::get<Point>(&(i->first))){
      const Point* p =  boost::get<Point>(&(i->first) );
      intersections_eigen_format.push_back(
          Eigen::Vector3d(p->x(), p->y(), p->z()));
    }
  }
  return intersections_eigen_format;
}

} // namespace COLLISION_DETECTION