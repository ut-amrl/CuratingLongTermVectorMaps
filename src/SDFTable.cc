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
  // TODO: Figure out how to do weighted average?
  weights_ = shortTermSDF.GetWeightDebugImage();
}

void SDFTable::filterCloud(const std::vector<Vector2f>& point_cloud, std::vector<Vector2f>& filtered_point_cloud) {
    double max_weight = weights_.max();
    // TODO: move to .h
    double T = 0.95 * max_weight;
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