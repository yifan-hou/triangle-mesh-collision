#ifndef __COLLISION_DETECT_H__
#define __COLLISION_DETECT_H__

#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <CGAL/Cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/IO/STL_reader.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/transform.h>

namespace COLLISION_DETECTION {

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;

class CollisionDetect{

  public:
    CollisionDetect(){};
    ~CollisionDetect(){};
    /**
     * Load model files into freaking CGAL format. CGAL only supports .off
     * format.
     *
     * @param[in]  filename1  The filename for object 1
     * @param[in]  filename2  The filename for object 2
     *
     * @return     true if both files are load successfully.
     */
    bool initialize(std::string filename1, std::string filename2);
    /**
     * Set the transformation for object one.
     *
     * @param[in]  R     The rotation matrix.
     * @param[in]  p     The translation
     */
    void setTransformation1(const Eigen::Matrix3d &R,
            const Eigen::Vector3d &p);
    void setTransformation2(const Eigen::Matrix3d &R,
            const Eigen::Vector3d &p);
    /**
     * Check if the transformed object 1 and 2 has collision.
     *
     * @return     true if a collision happens.
     */
    bool checkCollisions();

    void draw();
  private:
    Mesh mesh1_, mesh2_;
    Mesh mesh_transformed1_, mesh_transformed2_;
};

}

#endif
