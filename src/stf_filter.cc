#include <csignal>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include "ros/node_handle.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "Eigen/Dense"
#include "SDFTable.h"
#include "CImg.h"
#include <geometry_msgs/Pose.h>
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "pointcloud_helpers.h"

using std::string;
using std::vector;
using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

DEFINE_string(
  bag_file,
  "",
  "Bag file from which to read scans.");
DEFINE_string(
  lidar_topic,
  "/Cobot/Laser",
  "topic within bag file which to read scans.");
DEFINE_string(
  loc_topic,
  "/Cobot/Localization",
  "topic within bag file which to read locations.");
DEFINE_double(
  laser_range,
  30,
  "The maximum range of scans.");

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

void populateSDFTableFromBagFile(SDFTable& sdf, rosbag::Bag& bag) {
  // Get the topics we want
  vector<string> topics;
  topics.emplace_back(FLAGS_lidar_topic.c_str());
  topics.emplace_back(FLAGS_loc_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  printf("Bag file has %d scans\n", view.size());
  Vector2f lastLoc(0, 0);
  Eigen::Rotation2Df lastOrientation(0);
  // Iterate through the bag
  for (rosbag::View::iterator it = view.begin();
       it != view.end();
       ++it) {
    const rosbag::MessageInstance &message = *it;
    {
      sensor_msgs::LaserScanPtr laser_scan =
              message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        // Process the laser scan
        sdf.populateFromScan(*laser_scan, true, lastLoc, lastOrientation);
      }

      geometry_msgs::PosePtr pose =
              message.instantiate<geometry_msgs::Pose>();
      if (pose != nullptr) {
        // TODO general poses
        break;
      }

      cobot_msgs::CobotLocalizationMsgPtr loc = message.instantiate<cobot_msgs::CobotLocalizationMsg>();
      if (loc != nullptr) {
        lastLoc = Vector2f(loc->x, loc->y);
        lastOrientation = Eigen::Rotation2Df(loc->angle).toRotationMatrix();
      }
    }
  }
  sdf.normalizeWeights();
}

// Given 2 scans calculate relative transformation & uncertainty
void filter_short_term_features(string bag_path) {
  SDFTable sdf = SDFTable(400, 400, 0.25);

  printf("Loading bag file...\n");
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& exception) {
    printf("Unable to read %s, reason %s:", bag_path.c_str(), exception.what());
    return;
  }
  printf("Constructing SDF...\n");

  populateSDFTableFromBagFile(sdf, bag);

  cimg_library::CImgDisplay display1;
  display1.display(sdf.GetDistanceDebugImage().resize_tripleXY());
  
  rosbag::Bag writeBag;
  writeBag.open(bag_path + ".filtered", rosbag::bagmode::Write);

  printf("Filtering point clouds...\n");
  vector<string> topics;
  topics.emplace_back(FLAGS_lidar_topic.c_str());
  topics.emplace_back(FLAGS_loc_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<std::vector<Vector2f>> filteredClouds;
  Vector2f lastLoc(0, 0);
  Eigen::Rotation2Df lastOrientation(0);

  // Iterate through the bag, constructing filtered scans
  for (rosbag::View::iterator it = view.begin();
       it != view.end();
       ++it) {
    const rosbag::MessageInstance &message = *it;
    {
      sensor_msgs::LaserScanPtr laser_scan =
              message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        // Process the laser scan
        
        std::vector<Vector2f> cloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, FLAGS_laser_range, true, lastLoc, lastOrientation.toRotationMatrix());
        
        std::vector<Vector2f> filtered;
        sdf.filterCloud(cloud, filtered);

        PointCloud pcl_msg;

        pcl_msg.header.frame_id = "cloud";
        pcl_msg.height = 1;
        pcl_msg.width = filtered.size();

        for (auto point : filtered) {
          pcl_msg.points.push_back(pcl::PointXYZ(point.x(), point.y(), 0));
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pcl_msg, msg);

        writeBag.write("/filtered", ros::Time(laser_scan->header.stamp.sec + laser_scan->header.stamp.nsec*1e-9), msg);
        printf("Filtered scans has %lu points of %lu\n", filtered.size(), cloud.size());
        filteredClouds.push_back(filtered);
      }

      geometry_msgs::PosePtr pose =
              message.instantiate<geometry_msgs::Pose>();
      if (pose != nullptr) {
        // TODO general poses
        break;
      }

      cobot_msgs::CobotLocalizationMsgPtr loc = message.instantiate<cobot_msgs::CobotLocalizationMsg>();
      if (loc != nullptr) {
        lastLoc = Vector2f(loc->x, loc->y);
        lastOrientation = Eigen::Rotation2Df(loc->angle);
      }
    }
  }

  writeBag.close();
  bag.close();
  printf("Done.\n");
  fflush(stdout);

  while (!display1.is_closed()) {
    display1.wait();
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  if (FLAGS_bag_file.compare("") != 0) {
    filter_short_term_features(FLAGS_bag_file);
  } else {
    std::cout << "Must specify bag file, lidar topic" << std::endl;
    exit(0);
  }
  return 0;
}
