/*
  Implementation fo Bounding Volume Heirarchy (BVH) data struct
*/

#include "bvh.h"
#include <algorithm>
#include <assert.h>
#include <vector>

BVHNode::BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi nodeTris) {
  this->allV = allV;
  left = nullptr;
  right = nullptr;
  buildNode(nodeTris);
}

void BVHNode::buildNode(Eigen::MatrixXi nodeTris) {
  // If node is a leaf, save triangle/bounding box and exit 
  if (nodeTris.rows() == 1) {
    triangle = nodeTris.row(0);
    Eigen::MatrixXd points = triangleToPoints(triangle);
    boundingBox = new BoundingBox(points);
    return;
  } else {
    triangle = Eigen::RowVector4i(-1, -1, -1, -1);
  }

  // Compute bouding box for triangles (save to instance var)
  boundingBox = findBoundingBoxSet(nodeTris);

  // Determine longest length of bounding box, split into 2
  Eigen::MatrixXd minMax = boundingBox->getMinMax();

  double xDiff = minMax(1,0) - minMax(0,0);
  double yDiff = minMax(1,1) - minMax(0,1);
  double zDiff = minMax(1,2) - minMax(0,2);
  double maxDiff = std::max(xDiff, std::max(yDiff, zDiff));

  // Create 'bounding box' for the two split halves
  Eigen::MatrixXd boundSplitFirst(boundingBox->getMinMax());
  Eigen::MatrixXd boundSplitSec(boundingBox->getMinMax());
  if (maxDiff == xDiff) {
    boundSplitFirst(1,0) = xDiff/2;
    boundSplitSec(0,0) = xDiff/2;
  } else if (maxDiff == yDiff) {
    boundSplitFirst(1,1) = yDiff/2;
    boundSplitSec(0,1) = yDiff/2;
  } else if (maxDiff == zDiff) {
    boundSplitFirst(1,2) = zDiff/2;
    boundSplitSec(0,2) = zDiff/2;
  } else {
    // Should never happen! Problem with equality of doubles
    assert(false);
  }
  BoundingBox *boundFirst = new BoundingBox();
  boundFirst->minMax = boundSplitFirst;
  BoundingBox *boundSec = new BoundingBox();
  boundSec->minMax = boundSplitSec;

  // Partition triangles into 2 sets (depending on side of bounding box)
  std::vector<int> firstTriangleInd;
  std::vector<int> secTriangleInd;
  for (int i = 0; i < nodeTris.rows(); i++) {
    BoundingBox *currTriangle = 
      new BoundingBox(triangleToPoints(nodeTris.row(i)));
    if (currTriangle->intersectsWith(boundFirst)) {
      firstTriangleInd.push_back(i);
    }
    if (currTriangle->intersectsWith(boundSec)) {
      secTriangleInd.push_back(i);
    }
  }

  Eigen::MatrixXi firstTriangles(firstTriangleInd.size(), 4);
  Eigen::MatrixXi secTriangles(secTriangleInd.size(), 4);
  for (int i = 0; i < firstTriangleInd.size(); i++) {
    Eigen::RowVectorXi currTriangle = nodeTris.row(firstTriangleInd[i]);
    firstTriangles.block<1,4>(i,0) = currTriangle;
  }
  for (int i = 0; i < secTriangleInd.size(); i++) {
    Eigen::RowVectorXi currTriangle = nodeTris.row(firstTriangleInd[i]);
    secTriangles.block<1,4>(i,0) = currTriangle;
  }

  // Create left and right children
  left = new BVHNode(allV, firstTriangles);
  right = new BVHNode(allV, secTriangles);

  return;
}

//TODO: untested
Eigen::MatrixXd BVHNode::triangleToPoints(Eigen::Vector4i triangle) {
  Eigen::RowVector3d p1 = (*allV).block<1,3>(triangle(0), 0);
  Eigen::RowVector3d p2 = (*allV).block<1,3>(triangle(1), 0);
  Eigen::RowVector3d p3 = (*allV).block<1,3>(triangle(2), 0);
  Eigen::MatrixXd points(3, 3);
  points << p1, p2, p3;
  return points;
}

BoundingBox* BVHNode::findBoundingBoxSet(Eigen::MatrixXi triangles) {
  Eigen::MatrixXd points(triangles.rows()*3, 3);
  for (int i = 0; i < triangles.rows(); i++) {
      points.block<3, 3>(i*3, 0) = triangleToPoints(triangles.row(i));
  }
  return (new BoundingBox(points));
}


/*
    how to construct bvt:

- for each node (given a set of triangles):
    - check if leaf
    - construct bounding box. save
    - find longest length
    - divide bounding box by 2
    - partition triangles from set into the 2 regions
        - for each triangle in set, create bounding box and check collision with both
    - set left, right = recursive call
*/