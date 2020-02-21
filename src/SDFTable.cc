#include "./SDFTable.h"
#include "./Constants.h"


void SDFTable::populateFromScan(sensor_msgs::LaserScan& laser_scan, bool truncate_ends) {

  vector<pair<size_t, Vector2f>> pointcloud;
  float angle_offset = laser_scan.angle_min;
  float angle_truncation = truncate_ends ? M_PI / 12.0 : 0;
  for (size_t index = 0; index < laser_scan.ranges.size(); index++) {
    float range = laser_scan.ranges[index];
    if (range >= laser_scan.range_min && range <= max_range_ && angle_offset > laser_scan.angle_min + angle_truncation && angle_offset < laser_scan.angle_max - angle_truncation) {
      // Only accept valid ranges.
      // Then we must rotate the point by the specified angle at that distance.
      Eigen::Matrix2f rot_matrix =
      Eigen::Rotation2D<float>(angle_offset)
          .toRotationMatrix();
      // For every pixel in the map lying along the ray, update the values
      for(float sub_range = 0; sub_range < range + delta_; sub_range += resolution_) {
        Vector2f point(sub_range, 0.0);
        point = rot_matrix * point;

        double saved_distance = getPointDistance(point);
        double saved_weight = getPointWeight(point);
        double signed_dist = -(sub_range - range);
        double G = -sigma_ * pow(signed_dist - epsilon_, 2);
        
        double curr_d = ((signed_dist > delta_) ? delta_ : (abs(signed_dist) <= delta_) ? signed_dist : (signed_dist > -delta_) ? -delta_ : saved_distance);
        double curr_w = ((abs(signed_dist) < epsilon_) ? 1 : (abs(signed_dist) <= delta_) ? exp(G) : 0);

        double new_d = (saved_weight * saved_distance) + (curr_w * curr_d) / (saved_weight + curr_w);
        double new_w = (curr_w + saved_weight);

        setPointDistance(point, new_d);
        setPointWeight(point, new_w);
      }

    }
    angle_offset += laser_scan.angle_increment;
  }
}

void SDFTable::normalizeWeights() {
  double w_max = weights_.max();

  for(uint64_t x = 0; x < width_; x++) {
    for(uint64_t y = 0; y < width_; y++) {
      double w = getPointWeight(x, y);
      double target = (w / w_max) < Constants::T1 ? 0.0 : 1.0;
      setPointWeight(x, y, target);
    }
  }
}