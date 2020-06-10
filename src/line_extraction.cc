/*
 * Work Based on "Curating Long-Term Vector Maps"
 * by Samer Nashed and Joydeep Biswas.
 */

#include <time.h>
#include <mutex>
#include <thread>
#include <vector>

#include "Eigen/Dense"
#include "ceres/ceres.h"

#include "line_extraction.h"

using ceres::AutoDiffCostFunction;
using Eigen::Vector2f;
using std::make_pair;
using std::pair;
using std::sort;
using std::vector;

#define INLIER_THRESHOLD 0.05
#define CONVERGANCE_THRESHOLD 0.01
#define NEIGHBORHOOD_SIZE 0.25
#define NEIGHBORHOOD_GROWTH_SIZE 0.15
#define POINT_NUM_ACCEPTANCE_THRESHOLD 150

#define DEBUG false

namespace VectorMaps {

// Returns a list of inliers to the line_segment
// with their indices attached as the first member of the pair.
vector<pair<int, Vector2f>> GetInliers(const LineSegment line,
                                       const vector<Vector2f> pointcloud) {
  vector<pair<int, Vector2f>> inliers;
  for (size_t i = 0; i < pointcloud.size(); i++) {
    if (line.DistanceToLineSegment(pointcloud[i]) <= INLIER_THRESHOLD) {
      inliers.push_back(make_pair(i, pointcloud[i]));
    }
  }
  return inliers;
}

LineSegment RANSACLineSegment(const vector<Vector2f> pointcloud) {
  CHECK_GT(pointcloud.size(), 1);
  size_t max_possible_pairs = pointcloud.size() * (pointcloud.size() - 1);
  LineSegment best_segment;
  size_t max_inlier_num = 0;
  srand(time(NULL));
  // Loop through all pairs, or 1000 hundred of them if it's more than that
  for (size_t i = 0;
       i < std::min(max_possible_pairs, static_cast<size_t>(1000)); i++) {
    // Find all inliers between two different random points.
    size_t start_point_index = rand() % pointcloud.size();
    Vector2f start_point = pointcloud[start_point_index];
    Vector2f end_point;
    size_t end_point_index;
    do {
      end_point_index = rand() % pointcloud.size();
      end_point = pointcloud[end_point_index];
    } while (end_point_index == start_point_index);
    // Calculate the number of inliers, if more than best pair so far, save it.
    LineSegment line(start_point, end_point);
    size_t inlier_num = GetInliers(line, pointcloud).size();
    if (inlier_num > max_inlier_num) {
      max_inlier_num = inlier_num;
      best_segment = line;
    }
  }
  CHECK_GT(max_inlier_num, 0);
  return best_segment;
}

vector<Vector2f> GetNeighborhood(vector<Vector2f>& points) {
  // Pick a random point to center this around.
  vector<Vector2f> remaining_points = points;
  srand(time(NULL));
  vector<Vector2f> neighborhood;
  do {
    neighborhood.clear();
    if (remaining_points.size() <= 0) {
      return vector<Vector2f>();
    }
    size_t rand_idx = rand() % remaining_points.size();
    Vector2f center_point = remaining_points[rand_idx];
    // Now get the points around it
    for (Vector2f point : remaining_points) {
      if ((center_point - point).norm() <= NEIGHBORHOOD_SIZE) {
        neighborhood.push_back(point);
      }
    }
    if (neighborhood.size() <= 1) {
      // Remove the point both from the points we are searching in, and from the
      // original list, as its an outlier.
      bool found = false;
      for (size_t p_index = 0; p_index < points.size(); p_index++) {
        if (points[p_index].x() == center_point.x() &&
            points[p_index].y() == center_point.y()) {
          points.erase(points.begin() + p_index);
          remaining_points.erase(remaining_points.begin() + rand_idx);
          found = true;
          break;
        }
      }
      CHECK(found) << "Didn't find point in overall list of points.";
    }
  } while (neighborhood.size() <= 1);
  CHECK_GT(neighborhood.size(), 1);
  return neighborhood;
}

vector<Vector2f> GetNeighborhoodAroundLine(const LineSegment& line,
                                           const vector<Vector2f> points) {
  vector<Vector2f> neighborhood;
  for (const Vector2f& p : points) {
    double dist = line.DistanceToLineSegment(p);
    if (dist <= NEIGHBORHOOD_GROWTH_SIZE) {
      neighborhood.push_back(p);
    }
  }
  return neighborhood;
}

Vector2f GetCenterOfMass(const vector<Vector2f>& pointcloud) {
  Vector2f sum(0, 0);
  for (const Vector2f& p : pointcloud) {
    sum += p;
  }
  return sum / pointcloud.size();
}

std::mutex lock;

struct FitLineResidual {
  template <typename T>
  bool operator()(const T* first_point, const T* second_point,
                  T* residuals) const {
    typedef Eigen::Matrix<T, 2, 1> Vector2T;
    typedef Eigen::Hyperplane<T, 2> Line2T;

    // Cast everything over to generic
    const Vector2T first_pointT(first_point[0], first_point[1]);
    const Vector2T second_pointT(second_point[0], second_point[1]);
    const Vector2T pointT = point.cast<T>();
    T t;
    const Line2T infinite_line = Line2T::Through(first_pointT, second_pointT);
    Vector2T point_projection = infinite_line.projection(pointT);
    Vector2T diff_vec = second_pointT - first_pointT;
    if (abs(diff_vec.x()) > T(SAME_POINT_EPSILON)) {
      t = (point_projection.x() - first_pointT.x()) / diff_vec.x();
    } else if (abs(diff_vec.y()) > T(SAME_POINT_EPSILON)) {
      t = (point_projection.y() - first_pointT.y()) / diff_vec.y();
    } else {
      // The start and endpoints are so close that we should treat them as one
      // point. Return -1 so that the distance calculation will just return the
      // distance to one of the endpoints, either one, doesn't matter.
      t = T(-1);
    }
    Vector2T dist_vec;
    // If points are sufficiently close then its the same as being closer to the
    // start_point of the line.
    if (t < T(0)) {
      dist_vec = pointT - first_pointT;
    } else if (t > T(1)) {
      dist_vec = second_pointT - pointT;
    } else {
      Vector2T proj = first_pointT + t * (diff_vec);
      dist_vec = (pointT - proj);
    }
    if (!ceres::IsFinite(dist_vec.x()) || !ceres::IsFinite(dist_vec.y())) {
      lock.lock();
      std::cout << "P: " << point << std::endl;
      std::cout << "FP: " << first_point[0] << " " << first_point[1]
                << std::endl;
      std::cout << "SP: " << second_point[0] << " " << second_point[1]
                << std::endl;
      std::cout << "Dist: " << dist_vec.x() << " " << dist_vec.y() << std::endl;
      std::cout << "T: " << t << std::endl;
      std::cout << "StP: " << line_seg.start_point << std::endl;
      std::cout << "EnP: " << line_seg.end_point << std::endl;
      lock.unlock();
      exit(1);
    }
    residuals[0] = dist_vec.x();
    residuals[1] = dist_vec.y();
    return true;
  }

  FitLineResidual(const LineSegment& line_seg, const Vector2f point,
                  const Vector2f center_of_mass, const size_t point_num)
      : line_seg(line_seg),
        point(point),
        center_of_mass(center_of_mass),
        point_num(point_num) {}

  static AutoDiffCostFunction<FitLineResidual, 2, 2, 2>* create(
      const LineSegment& line_seg, const Vector2f point,
      const Vector2f center_of_mass, const size_t point_num) {
    FitLineResidual* line_res =
        new FitLineResidual(line_seg, point, center_of_mass, point_num);
    return new AutoDiffCostFunction<FitLineResidual, 2, 2, 2>(line_res);
  }

  const LineSegment line_seg;
  const Vector2f point;
  const Vector2f center_of_mass;
  const size_t point_num;
};

double FindMaxDistance(const vector<Vector2f>& points) {
  double max_dist = 0.0;
  for (const Vector2f& p1 : points) {
    for (const Vector2f& p2 : points) {
      if ((p1 - p2).norm() > max_dist) {
        max_dist = (p1 - p2).norm();
      }
    }
  }
  return max_dist;
}

LineSegment FitLine(LineSegment line, const vector<Vector2f> pointcloud) {
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  ceres::Problem problem;
  double first_point[] = {line.start_point.x(), line.start_point.y()};
  double second_point[] = {line.end_point.x(), line.end_point.y()};
  Vector2f center_of_mass = GetCenterOfMass(pointcloud);
  size_t point_num = pointcloud.size();
  for (const Vector2f& point : pointcloud) {
    problem.AddResidualBlock(
        FitLineResidual::create(line, point, center_of_mass, point_num), NULL,
        first_point, second_point);
  }
  ceres::Solve(options, &problem, &summary);
  Vector2f new_start_point(first_point[0], first_point[1]);
  Vector2f new_end_point(second_point[0], second_point[1]);
  return LineSegment(new_start_point, new_end_point);
}

vector<Vector2f> GetPointsFromInliers(vector<pair<int, Vector2f>> inliers) {
  vector<Vector2f> points;
  for (const pair<int, Vector2f> p : inliers) {
    points.push_back(p.second);
  }
  return points;
}

vector<Vector2f> GetPointsOnLine(const LineSegment& line,
                                 const vector<Vector2f>& points) {
  vector<Vector2f> points_on_line;
  for (const Vector2f& p : points) {
    if (line.PointOnLine(p, INLIER_THRESHOLD)) {
      points_on_line.push_back(p);
    }
  }
  return points_on_line;
}

Vector2f GetClosestPoint(const Vector2f vec, const vector<Vector2f>& points) {
  CHECK_GT(points.size(), 0);
  Vector2f closest = points[0];
  for (const Vector2f& p : points) {
    if ((vec - p).norm() < (vec - closest).norm()) {
      closest = p;
    }
  }
  return closest;
}

LineSegment ClipLineToPoints(const LineSegment& line,
                             const vector<Vector2f>& points) {
  const vector<Vector2f> points_on_line = GetPointsOnLine(line, points);
  if (points_on_line.size() == 0) {
    return line;
  }
  const Vector2f& closest_to_start =
      GetClosestPoint(line.start_point, points_on_line);
  const Vector2f& closest_to_end =
      GetClosestPoint(line.end_point, points_on_line);
  // Project onto the line.
  Eigen::Hyperplane<float, 2> infinite_line =
      Eigen::Hyperplane<float, 2>::Through(line.start_point, line.end_point);
  const Vector2f start_proj = infinite_line.projection(closest_to_start);
  const Vector2f end_proj = infinite_line.projection(closest_to_end);
  return LineSegment(start_proj, end_proj);
}

// Returns a random pointcloud that is X percent of the
// specified pointcloud.
vector<Vector2f> DownsamplePointcloud(const vector<Vector2f>& pointcloud,
                                      const double size_reduction_percent) {
  // Used for small optimization so we don't have to delete points.
  int percentage_times_hundred = size_reduction_percent * 100;
  vector<Vector2f> downsampled_cloud;
  for (const Vector2f& point : pointcloud) {
    int rand_num = rand() % 100;
    if (rand_num <= percentage_times_hundred) {
      downsampled_cloud.push_back(point);
    }
  }
  return downsampled_cloud;
}

vector<LineSegment> ExtractLines(const vector<Vector2f>& pointcloud) {
  if (pointcloud.size() <= 1) {
    return vector<LineSegment>();
  }
  std::cout << "Original Point Cloud size: " << pointcloud.size() << std::endl;
  vector<Vector2f> remaining_points = DownsamplePointcloud(pointcloud, 0.80);
  std::cout << "Downsampled Point Cloud size: " << remaining_points.size()
            << std::endl;
  vector<LineSegment> lines;
  size_t stopping_threshold =
      std::max(static_cast<size_t>(pointcloud.size() * 0.03),
               static_cast<size_t>(POINT_NUM_ACCEPTANCE_THRESHOLD));
  while (remaining_points.size() >= stopping_threshold) {
    #if DEBUG
    std::cout << "Remaining Points : " << remaining_points.size() << std::endl;
    #endif
    // Restrict the RANSAC implementation to using a small subset of the points.
    // This will speed it up.
    vector<Vector2f> neighborhood = GetNeighborhood(remaining_points);
    if (neighborhood.size() <= 0) {
      break;
    }
    LineSegment line = RANSACLineSegment(neighborhood);
    LineSegment new_line = FitLine(line, neighborhood);
    new_line = ClipLineToPoints(new_line, neighborhood);
    vector<pair<int, Vector2f>> inliers;
    // Continually grow the line until it no longer gains more inliers.
    do {
      line = new_line;
      std::vector<Vector2f> neighborhood_to_consider =
          GetNeighborhoodAroundLine(new_line, remaining_points);
      if (neighborhood_to_consider.size() <= neighborhood.size()) {
        #if DEBUG
        std::cout << "Stopping b/c of neighborhood expansion failure"
                  << std::endl;
        #endif
        break;
      }
      LineSegment test_line = FitLine(new_line, neighborhood_to_consider);
      if (GetPointsOnLine(test_line, neighborhood).size() > 0) {
        // Make sure the line doesn't extend past the points it is fit too.
        test_line = ClipLineToPoints(test_line, neighborhood_to_consider);
        // Only grow if we gain points on the line.
        if (GetInliers(test_line, neighborhood_to_consider).size() >
            GetInliers(new_line, neighborhood_to_consider).size()) {
          new_line = test_line;
        } else {
          #if DEBUG
          std::cout << "Stopping b/c line growth inlier failure" << std::endl;
          #endif
        }
      } else {
        #if DEBUG
        std::cout << "Stopping b/c of line growth failure" << std::endl;
        #endif
      }
      // Converge once the line doesn't move a lot.
      #if DEBUG
      std::cout << "Change Amount: "
                << (new_line.start_point - line.start_point).norm() +
                       (new_line.end_point - line.end_point).norm()
                << std::endl;
      #endif
    } while ((new_line.start_point - line.start_point).norm() +
                 (new_line.end_point - line.end_point).norm() >
             CONVERGANCE_THRESHOLD);
    inliers = GetInliers(new_line, remaining_points);
    if (inliers.size() >= POINT_NUM_ACCEPTANCE_THRESHOLD) {
      lines.push_back(new_line);
    }
    // We have to remove the points that were assigned to this line.
    // Sort the inliers by their index so we don't get weird index problems.
    // We should also remove the points that belong to small lines, so that we
    // don't use them again.
    #if DEBUG
    std::cout << "Sorting" << std::endl;
    #endif
    sort(inliers.begin(), inliers.end(),
         [](pair<int, Vector2f> p1, pair<int, Vector2f> p2) {
           return p1.first < p2.first;
         });
    #if DEBUG
    std::cout << "Beginning Deletion" << std::endl;
    #endif
    // Erasing them is a very expensive operation, so instead lets do a one
    // pass.
    vector<Vector2f> new_remaining_points;
    size_t last_inlier_index = -1;
    for (size_t inlier_index = 0; inlier_index < inliers.size();
         inlier_index++) {
      CHECK_LT(inliers[inlier_index].first, remaining_points.size());
      for (int point_index = last_inlier_index + 1;
           point_index < inliers[inlier_index].first; point_index++) {
        new_remaining_points.push_back(remaining_points[point_index]);
      }
      last_inlier_index = inliers[inlier_index].first;
    }
    for (size_t point_index = last_inlier_index + 1;
         point_index < remaining_points.size(); point_index++) {
      new_remaining_points.push_back(remaining_points[point_index]);
    }
    CHECK_EQ(new_remaining_points.size(),
             remaining_points.size() - inliers.size());
    remaining_points = new_remaining_points;
  }
  #if DEBUG
  std::cout << std::endl;
  std::cout << "Pointcloud size: " << pointcloud.size() << std::endl;
  std::cout << "Lines size: " << lines.size() << std::endl;
  #endif
  return lines;
}

}  // namespace VectorMaps
