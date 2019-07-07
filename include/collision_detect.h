#ifndef __COLLISION_DETECT_H__
#define __COLLISION_DETECT_H__

#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/IO/STL_reader.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/transform.h>

namespace COLLISION_DETECTION {

  typedef CGAL::Simple_cartesian<double> K;
  typedef K::FT FT;
  typedef K::Point_3 Point;
  typedef K::Vector_3 Vector;
  typedef K::Ray_3 Ray;
  typedef CGAL::Surface_mesh<Point> Mesh;
  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef std::shared_ptr<Tree> TreePtr;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

  /**
   * @brief      Collision detection between two triangle meshes.
   */
  class CollisionDetect{

  public:
    CollisionDetect(){};
    ~CollisionDetect(){};
    /**
     * Load model files into the freaking CGAL format. CGAL only supports .off
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

/**
 * @brief      Ray tracing on a mesh.
 *             https://doc.cgal.org/latest/AABB_tree/index.html
 */
  class RayTracing{

  public:
    RayTracing(){};
    ~RayTracing(){};
    /**
     * Load model files into the freaking CGAL format. Build an AABB tree.
     * CGAL only supports .off format.
     *
     * @param[in]  filename  The filename of the object.
     *
     * @return     true if both files are load successfully.
     */
    bool initialize(std::string filename);
    /**
     * Compute a list of intersection points between the mesh and a given ray.
     *
     * @param[in]  p     Origin of the ray.
     * @param[in]  n     Direction of the ray. Must not be of zero length
     *
     * @return     true if a collision happens.
     */
    std::vector<Eigen::Vector3d> findIntersections(
        const Eigen::Vector3d &p, const Eigen::Vector3d &n);

  private:
    Mesh mesh_;
    // todo: use tree_ as member variable. Currently with CGAL 4.14, this will cause weird bugs
    // Tree tree_;
  };

}

#endif
