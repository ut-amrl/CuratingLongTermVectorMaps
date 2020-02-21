#ifndef SRC_SDFTABLE_H
#define SRC_SDFTABLE_H

#include <glog/logging.h>
#include <cstdint>
#include <cmath>

#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

#include "./CImg.h"

using std::vector;
using std::pair;
using sensor_msgs::Image;
using cimg_library::CImg;

class SDFTable {
  uint64_t width_;
  uint64_t height_;
  double max_range_;
  double resolution_;
  double delta_;
  double epsilon_;
  double sigma_;
  CImg<double> distances_;
  CImg<double> weights_;
  SDFTable(const double range,
              const double resolution, const double delta = 0.1, const double epsilon = 0.2, const double sigma = 0.5) :
              width_(floor((range * 2.0) / resolution)),
              height_(floor((range * 2.0) / resolution)),
              max_range_(range),
              resolution_(resolution),
              delta_(delta),
              epsilon_(epsilon),
              sigma_(sigma) {
    // Construct a width x height image, with only 1 z level.
    // And, only one double per color with default value 0.0.
    distances_ = CImg<double>(width, height, 1, 1, 0.0);
    weights_ = CImg<double>(width, height, 1, 1, 0.0);
  }

  SDFTable() : width_(0), height_(0), max_range_(0), resolution_(1) {}

  inline uint64_t convertX(float x) const {
    return width_ / 2 + floor(x / resolution);
  }

  inline uint64_t convertY(float y) const {
    return height_ / 2 + floor(y / resolution);
  }

  inline double getPointDistance(Vector2f point) const {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    return distances_(x, y);
  }

  CImg<double> GetDistanceDebugImage() const {
    return distances_;
  }

  inline double getPointWeight(Vector2f point) const {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    return weights_(x, y);
  }

  CImg<double> GetWeightDebugImage() const {
    return weights_;
  }
  
  void populateFromScan(sensor_msgs::LaserScan &laser_scan, bool truncate_ends);
};

#endif  // SRC_SDFTABLE_H
