//
// Created by jack on 2/21/20.
//

#include <time.h>
#include <vector>
#include <thread>
#include <mutex>

#include "Eigen/Dense"
#include "ceres/ceres.h"

#include "kdtree.h"
#include "line_extraction.h"

using std::vector;
using std::pair;
using std::make_pair;
using std::sort;
using Eigen::Vector2f;
using ceres::AutoDiffCostFunction;

#define INLIER_THRESHOLD 0.05
#define CONVERGANCE_THRESHOLD 0.005
#define NEIGHBORHOOD_SIZE 0.25

namespace VectorMaps {

// Returns a list of inliers to the line_segment
// with their indices attached as the first member of the pair.
vector<pair<int, Vector2f>> GetInliers(const LineSegment line,
                                       const vector<Vector2f> pointcloud) {
  vector<pair<int, Vector2f>> inliers;
  for (size_t i = 0; i < pointcloud.size(); i++) {
    if (line.DistanceToLineSegment(pointcloud[i]) < INLIER_THRESHOLD) {
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
  for (size_t i = 0;
       i < std::min(max_possible_pairs, static_cast<size_t>(100));
       i++) {
    Vector2f start_point = pointcloud[rand() % pointcloud.size()];
    Vector2f end_point;
    do {
      end_point = pointcloud[rand() % pointcloud.size()];
    } while (start_point == end_point);
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

vector<Vector2f> GetNeighborhood(const vector<Vector2f> points) {
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
      remaining_points.erase(remaining_points.begin() + rand_idx);
    }
  } while(neighborhood.size() <= 1);
  CHECK_GT(neighborhood.size(), 1);
  return neighborhood;
}

Vector2f GetCenterOfMass(const vector<Vector2f>& pointcloud) {
  Vector2f sum(0,0);
  for (const Vector2f& p : pointcloud) {
    sum += p;
  }
  return sum / pointcloud.size();
}

std::mutex lock;

struct FitLineResidual {
    template<typename T>
    bool operator() (const T* first_point,
                     const T* second_point,
                     T* residuals) const {
      typedef Eigen::Matrix<T, 2, 1> Vector2T;
      // Cast everything over to generic
      const Vector2T first_pointT(first_point[0], first_point[1]);
      const Vector2T second_pointT(second_point[0], second_point[1]);
      const Vector2T pointT = point.cast<T>();
      //Vector2T center = center_of_mass.cast<T>();
      Vector2T line_seg_start = line_seg.start_point.cast<T>();
      Vector2T line_seg_end = line_seg.end_point.cast<T>();
      // Find the centroid (missing denominator which is used later).
      T r = T((center_of_mass - line_seg.start_point).norm() +
            (center_of_mass - line_seg.end_point).norm());
      T t;
      if (abs((second_pointT - first_pointT).norm()) < 0.0001) {
        // If the two points are basically the same point, then t should be 0 so later R will be the distance
        // from the first_point to the pointT.
        t = T(0);
      } else {
        t = (pointT - first_pointT).dot(second_pointT - first_pointT) /
            (second_pointT - first_pointT).norm();
      }
      T dist;
      if (t < T(0)) {
        dist = (pointT - line_seg_start).norm();
      } else if (t > T(1)) {
        dist = (pointT - line_seg_end).norm();
      } else {
        //dist = (pointT - first_pointT + t * (second_pointT - first_pointT)).norm();
        Eigen::Hyperplane<T, 2> line = Eigen::Hyperplane<T, 2>::Through(first_pointT, second_pointT);
        dist = line.absDistance(pointT);
      }
      if (!ceres::isfinite((r / T(point_num)) + dist)) {
        lock.lock();
        std::cout << "R: " << r << std::endl;
        std::cout << "P: " << point << std::endl;
        std::cout << "FP: " << first_point[0] << " " << first_point[1] << std::endl;
        std::cout << "SP: " << second_point[0] << " " << second_point[1] << std::endl;
        std::cout << "Dist: " << dist << std::endl;
        std::cout << "T: " << t << std::endl;
        std::cout << "StP: " << line_seg.start_point << std::endl;
        std::cout << "EnP: " << line_seg.end_point << std::endl;
        lock.unlock();
        exit(1);
      }
      residuals[0] = (r / T(point_num)) + dist;
      return true;
    }

    FitLineResidual(const LineSegment& line_seg,
                    const Vector2f point,
                    const Vector2f center_of_mass,
                    const size_t point_num) :
                    line_seg(line_seg),
                    point(point),
                    center_of_mass(center_of_mass),
                    point_num(point_num) {}

    static AutoDiffCostFunction<FitLineResidual, 1, 2, 2>*
    create(const LineSegment& line_seg,
           const Vector2f point,
           const Vector2f center_of_mass,
           const size_t point_num) {
      FitLineResidual *line_res = new FitLineResidual(line_seg, point, center_of_mass, point_num);
      return new AutoDiffCostFunction<FitLineResidual, 1, 2, 2>(line_res);
    }

    const LineSegment line_seg;
    const Vector2f point;
    const Vector2f center_of_mass;
    const size_t point_num;
};

LineSegment FitLine(LineSegment line, const vector<Vector2f> pointcloud) {
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_QR; // TODO: Testing to see if this is right.
  options.minimizer_progress_to_stdout = false;
  options.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  ceres::Problem problem;
  double first_point[] = {line.start_point.x(), line.start_point.y()};
  double second_point[] = {line.end_point.x(), line.end_point.y()};
  Vector2f center_of_mass = GetCenterOfMass(pointcloud);
  size_t point_num = pointcloud.size();
  for (const Vector2f& point : pointcloud) {
    problem.AddResidualBlock(FitLineResidual::create(line,
                                                     point,
                                                     center_of_mass,
                                                     point_num),
                             NULL,
                             first_point,
                             second_point);
  }
  ceres::Solve(options, &problem, &summary);
  Vector2f new_start_point(first_point[0], first_point[1]);
  Vector2f new_end_point(second_point[0], second_point[1]);
  return LineSegment(new_start_point, new_end_point);
}

vector <LineSegment> ExtractLines(const vector <Vector2f>& pointcloud) {
  vector<Vector2f> remaining_points = pointcloud;
  vector<LineSegment> lines;
  while (remaining_points.size() > 1) {
    // Restrict the RANSAC implementation to using a small subset of the points.
    // This will speed it up.
    vector<Vector2f> neighborhood = GetNeighborhood(remaining_points);
    if (neighborhood.size() == 0) {
      break;
    }
    std::cout << "Running RANSAC" << std::endl;
    LineSegment line = RANSACLineSegment(neighborhood);
    std::cout << "Get Inliers" << std::endl;
    vector<pair<int, Vector2f>> inliers = GetInliers(line, neighborhood);
    LineSegment new_line = FitLine(line, neighborhood);
    std::cout << "Fitting Line" << std::endl;
    while ((new_line.start_point - line.start_point).norm() +
           (new_line.end_point - line.end_point).norm() > CONVERGANCE_THRESHOLD) {
      inliers = GetInliers(new_line, neighborhood);
      line = new_line;
      new_line = FitLine(line, remaining_points);
    }
    lines.push_back(new_line);
    // We have to remove the points that were assigned to this line.
    // Sort the inliers by their index so we don't get weird index problems.
    inliers = GetInliers(new_line, remaining_points);
    sort(inliers.begin(), inliers.end(), [](pair<int, Vector2f> p1,
                                            pair<int, Vector2f> p2) {
      return p1.first < p2.first;
    });
    CHECK_GE(inliers.size(), 2);
    std::cout << "Removing: " << inliers.size() << std::endl;
    for (int64_t i = inliers.size() - 1; i >= 0; i--) {
      remaining_points.erase(remaining_points.begin() + inliers[i].first);
    }
    std::cout << "Points Remaining: " << remaining_points.size() << std::endl;
  }
  return lines;
}

}