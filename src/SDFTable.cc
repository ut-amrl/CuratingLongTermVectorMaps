#include "./SDFTable.h"
#include "./Constants.h"

using namespace Eigen;

void SDFTable::updateWeightAndDistance(Vector2f point, double signed_dist) {
    double saved_distance = getPointDistance(point);
    double saved_weight = getPointWeight(point);
    double G = -Constants::sigma * pow(signed_dist - Constants::epsilon, 2);
    
    double curr_d = ((signed_dist > Constants::delta) ? Constants::delta : (abs(signed_dist) <= Constants::delta) ? signed_dist : (signed_dist > -Constants::delta) ? -Constants::delta : saved_distance);
    double curr_w = ((abs(signed_dist) < Constants::epsilon) ? 1 : (abs(signed_dist) <= Constants::delta) ? exp(G) : 0);
    
    double new_w = (curr_w + saved_weight);
    if (new_w > 0) {
      double new_d = ((saved_weight * saved_distance) + (curr_w * curr_d)) / (saved_weight + curr_w);
      setPointDistance(point, new_d);
      // printf("Distances: %f\t%f\t%f\t%f\t\t", signed_dist, saved_distance, curr_d, new_d);
      // printf("Weights: %f\t%f\t%f\t\n", saved_weight, curr_w, new_w);
    }

    setPointWeight(point, new_w);
}

void SDFTable::populateFromScan(sensor_msgs::LaserScan& laser_scan, bool truncate_ends, Vector2f location, Rotation2Df orientation) {
  vector<pair<size_t, Vector2f>> pointcloud;
  float angle_offset = laser_scan.angle_min;
  float angle_truncation = truncate_ends ? M_PI / 12.0 : 0;

  for (size_t index = 0; index < laser_scan.ranges.size(); index++) {
    float range = laser_scan.ranges[index];
    if (range >= laser_scan.range_min && range <= laser_scan.range_max && angle_offset > laser_scan.angle_min + angle_truncation && angle_offset < laser_scan.angle_max - angle_truncation) {
      // Only accept valid ranges.
      // Then we must rotate the point by the specified angle at that distance.
      Matrix2f rot_matrix =
      Rotation2D<float>(angle_offset)
          .toRotationMatrix();
      // For every pixel in the map lying along the ray, update the values
      for(float sub_range = 0; sub_range < range + Constants::delta; sub_range += resolution_) {
        Vector2f point(sub_range, 0.0);
        point = rot_matrix * point;

        Vector2f global_point = orientation * (point) + location;

        double signed_dist = -(sub_range - range);
        updateWeightAndDistance(global_point, signed_dist);
      }

    }
    angle_offset += laser_scan.angle_increment;
  }
}

void SDFTable::populateFromScan(sensor_msgs::LaserScan& laser_scan, bool truncate_ends) {
  populateFromScan(laser_scan, truncate_ends, Vector2f(0, 0), Rotation2Df(0));
}

void SDFTable::normalizeWeights() {
  double w_max = weights_.max();

  for(uint64_t x = 0; x < width_; x++) {
    for(uint64_t y = 0; y < height_; y++) {
      double w = getPointWeight(x, y);
      double target = (w / w_max) < Constants::T1 ? 0.0 : 1.0;
      setPointWeight(x, y, target);
    }
  }
}

void SDFTable::updateWithSDF(SDFTable& shortTermSDF) {
  for(uint64_t x = 0; x < width_; x++) {
    for(uint64_t y = 0; y < height_; y++) {
      double w = getPointWeight(x, y);
      double d = getPointDistance(x, y);
      double other_w = shortTermSDF.getPointWeight(x, y);
      if (other_w > 0) {
        double other_d = shortTermSDF.getPointDistance(x, y);
        
        double new_w = w + other_w;
        double new_d = (w*d + other_w * other_d) / (w + other_w);

        setPointWeight(x, y, new_w);
        setPointDistance(x, y, new_d);
      }
    }
  }
}

void SDFTable::filterCloud(const std::vector<Vector2f>& point_cloud, std::vector<Vector2f>& filtered_point_cloud) {
    double max_weight = weights_.max();
    // TODO: move to .h
    double T = 0.9 * max_weight;
    double D = 0.05;

    for (size_t j = 0; j < point_cloud.size(); ++j) {
      const Vector2f point = point_cloud[j];

      double weight = this->getPointWeight(point);
      double distance = this->getPointDistance(point);

      if (weight > T && fabs(distance) < D) {
        filtered_point_cloud.push_back(point);
      }
    }
  }

  void SDFTable::filterScan(sensor_msgs::LaserScan& laser_scan, sensor_msgs::LaserScanPtr filtered_scan, bool truncate_ends, Vector2f location, Rotation2Df orientation) {
    
    double max_weight = weights_.max();
    // TODO: move to .h
    double T = 0.9 * max_weight;
    double D = 0.05;
    
    float angle_offset = laser_scan.angle_min;
    float angle_truncation = truncate_ends ? M_PI / 12.0 : 0;

    // copy the scan metadata
    filtered_scan->angle_min = laser_scan.angle_min;
    filtered_scan->angle_max = laser_scan.angle_max;
    filtered_scan->angle_increment = laser_scan.angle_increment;
    filtered_scan->time_increment = laser_scan.time_increment;
    filtered_scan->scan_time = laser_scan.scan_time;
    filtered_scan->range_min = laser_scan.range_min;
    filtered_scan->range_max = laser_scan.range_max;
    filtered_scan->header.seq = laser_scan.header.seq;
    filtered_scan->header.stamp = laser_scan.header.stamp;
    filtered_scan->header.frame_id = laser_scan.header.frame_id;
    filtered_scan->ranges.resize(laser_scan.ranges.size());

    for (size_t index = 0; index < laser_scan.ranges.size(); index++) {
      float range = laser_scan.ranges[index];

      if (range >= laser_scan.range_min && range <= laser_scan.range_max && angle_offset > laser_scan.angle_min + angle_truncation && angle_offset < laser_scan.angle_max - angle_truncation) {
        Matrix2f rot_matrix =
        Rotation2D<float>(angle_offset)
            .toRotationMatrix();
        Vector2f point(range, 0.0);
        point = rot_matrix * point;

        Vector2f global_point = orientation * (point) + location;

        double weight = this->getPointWeight(global_point);
        double distance = this->getPointDistance(global_point);

        if (weight > T && fabs(distance) < D) {
          filtered_scan->ranges[index] = range;
        } else {
          filtered_scan->ranges[index] = -1;
        }
      }

      angle_offset += laser_scan.angle_increment;
    }
  }