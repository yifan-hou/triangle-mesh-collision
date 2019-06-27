#include <string>
#include <iostream>
#include <fstream>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Cartesian.h>
#include <CGAL/IO/STL_reader.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <ctime>
#include <Eigen/Dense>

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

int main(int argc, char *argv[])
{
  const char* filename1 = "../meshes/BIG_SCREW.off";
  std::ifstream input1(filename1);
  Mesh mesh1;
  if (!input1 || !(input1 >> mesh1) || !CGAL::is_triangle_mesh(mesh1))
  {
    std::cerr << "Not a valid input file." << std::endl;
    return 1;
  }

  const char* filename2 = "../meshes/fingertip_8faces.off";
  std::ifstream input2(filename2);
  Mesh mesh2;
  if (!input2 || !(input2 >> mesh2) || !CGAL::is_triangle_mesh(mesh2))
  {
    std::cerr << "Not a valid input file." << std::endl;
    return 1;
  }

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  for (int i = 0; i < 10; ++i) {
    double begin = std::clock();
    Eigen::Vector3d p;
    p << 0, 2*i, 0;
    K::Aff_transformation_3 transform_cgal(
        R(0,0), R(0,1), R(0,2), p(0),
        R(1,0), R(1,1), R(1,2), p(1),
        R(2,0), R(2,1), R(2,2), p(2), 1);
    // K::Aff_transformation_3 transform_cgal(CGAL::TRANSLATION, K::Vector_3(0,i*2,0));
    PMP::transform (transform_cgal, mesh1);
    bool intersecting = PMP::do_intersect (mesh1, mesh2);
    std::cout << "Computation Time: " << (std::clock() - begin) / CLOCKS_PER_SEC << " sec" << std::endl;
    std::cout << "intersecting: " << intersecting << std::endl;
  }
}
