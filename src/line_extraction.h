//
// Created by jack on 2/21/20.
//

#ifndef CURATINGLONGTERMVECTORMAPS_LINE_EXTRACTION_H
#define CURATINGLONGTERMVECTORMAPS_LINE_EXTRACTION_H

#include <vector>

#include "Eigen/Dense"

using std::vector;
using Eigen::Vector2f;

namespace VectorMaps {

struct LineSegment {
    Vector2f start_point;
    Vector2f end_point;

    LineSegment(Vector2f start_point, Vector2f end_point) :
      start_point(start_point),
      end_point(end_point) {}

    LineSegment() {}

    double DistanceToLineSegment(Vector2f point) const {
      // Project the point onto the line.
      typedef Eigen::Hyperplane<float, 2> Line2D;
      const Line2D infinite_line = Line2D::Through(start_point, end_point);
      const Vector2f point_projection = infinite_line.projection(point);
      // Parameterize according to the projection.
      double t =
        (point_projection - start_point).norm() /
        (end_point - start_point).norm();
      if (t < 0) {
        // This point is closest to the start point.
        return (start_point - point).norm();
      } else if (t > 1) {
        // This point is closest to the end point.
        return (point - end_point).norm();
      } else {
        // Otherwise, the point is closest to the line.
        // This is equivalent to the orthogonal projection.
        return (point - point_projection).norm();
      }
    }
};

vector<LineSegment> ExtractLines(vector<Vector2f> pointcloud);

}

#endif //CURATINGLONGTERMVECTORMAPS_LINE_EXTRACTION_H
