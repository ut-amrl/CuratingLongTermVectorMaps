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
using cimg_library::CImg;

class SDFTable {
  uint64_t width_;
  uint64_t height_;
  double resolution_;
  CImg<double> distances_;
  CImg<double> weights_;

  protected:
    inline uint64_t convertX(float x) const {
      return width_ / 2 + floor(x / resolution_);
    }

    inline uint64_t convertY(float y) const {
      return height_ / 2 + floor(y / resolution_);
    }


    inline void setPointDistance(Vector2f point, double dist) {
      uint64_t x = convertX(point.x());
      uint64_t y = convertY(point.y());
      distances_(x, y) = dist;
    }

    inline void setPointWeight(Vector2f point, double w) {
      uint64_t x = convertX(point.x());
      uint64_t y = convertY(point.y());
      weights_(x, y) = w;
    }

    inline double getPointWeight(int x, int y) const {
      return weights_(x, y);
    }

    inline void setPointWeight(int x, int y, double w) {
      weights_(x, y) = w;
    }

  public:
    SDFTable(const uint64_t width, // width in pixels
                const uint64_t height, // height in pixels
                const double resolution) :
                width_(width),
                height_(height),
                resolution_(resolution) {
      // Construct a width x height image, with only 1 z level.
      // And, only one double per color with default value 0.0.
      distances_ = CImg<double>(width_, height_, 1, 1, 0.0);
      weights_ = CImg<double>(width_, height_, 1, 1, 0.0);
    }

    SDFTable(const double range, const double resolution) :
      SDFTable(floor((range * 2.0) / resolution), floor((range * 2.0) / resolution), resolution) {};

    SDFTable() : width_(0), height_(0), resolution_(1) {}

    inline double getPointDistance(Vector2f point) const {
      uint64_t x = convertX(point.x());
      uint64_t y = convertY(point.y());
      if (x >= width_ || y >= height_) {
        printf("BIG BAD (%f, %f) %lu %lu\n", point.x(), point.y(), x, y);
      }
      return distances_(x, y);
    }

    inline double getPointWeight(Vector2f point) const {
      uint64_t x = convertX(point.x());
      uint64_t y = convertY(point.y());
      if (x >= width_ || y >= height_) {
        printf("BIG BAD (%f, %f) %lu %lu\n", point.x(), point.y(), x, y);
      }
      return weights_(x, y);
    }

    CImg<double> GetWeightDebugImage() const {
      return weights_;
    }

    CImg<double> GetDistanceDebugImage() const {
      return distances_;
    }

    inline void clear() {
      weights_.clear();
      distances_.clear();
    }
    
    void updateWeightAndDistance(Vector2f point, double signed_distance);
    void populateFromScan(sensor_msgs::LaserScan &laser_scan, bool truncate_ends);
    void populateFromScan(sensor_msgs::LaserScan &laser_scan, bool truncate_ends, Eigen::Vector2f location, Eigen::Rotation2Df orientation);
    void normalizeWeights();
    void updateWithSDF(SDFTable &shortTermSDF);
};

#endif  // SRC_SDFTABLE_H
