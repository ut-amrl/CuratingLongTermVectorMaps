#include <algorithm>
#include <csignal>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/node_handle.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <geometry_msgs/Pose.h>
#include "CImg.h"
#include "Eigen/Dense"
#include "SDFTable.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "pointcloud_helpers.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

using std::string;
using std::vector;
using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud;

DEFINE_string(bag_file, "", "Bag file from which to read scans.");
DEFINE_string(
    bag_folder, "",
    "Bag folder from which to find multiple bag files containing scans.");
DEFINE_string(lidar_topic, "/Cobot/Laser",
              "topic within bag file which to read scans.");
DEFINE_string(loc_topic, "/Cobot/Localization",
              "topic within bag file which to read locations.");
DEFINE_double(laser_range, 30, "The maximum range of scans.");

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
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    {
      sensor_msgs::LaserScanPtr laser_scan =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        // Process the laser scan
        sdf.populateFromScan(*laser_scan, true, lastLoc, lastOrientation);
      }

      geometry_msgs::PosePtr pose = message.instantiate<geometry_msgs::Pose>();
      if (pose != nullptr) {
        // TODO general poses
        break;
      }

      cobot_msgs::CobotLocalizationMsgPtr loc =
          message.instantiate<cobot_msgs::CobotLocalizationMsg>();
      if (loc != nullptr) {
        lastLoc = Vector2f(loc->x, loc->y);
        lastOrientation = Eigen::Rotation2Df(loc->angle).toRotationMatrix();
      }
    }
  }
}

SDFTable construct_ltsdf(string bag_folder) {
  std::vector<SDFTable> stsdfs;
  for (const auto& entry : boost::filesystem::directory_iterator(bag_folder)) {
    string bag_path = entry.path().string();
    SDFTable sdf = SDFTable(1000, 1000, 0.1);

    printf("Loading bag file...\n");
    rosbag::Bag bag;
    try {
      bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException& exception) {
      printf("Unable to read %s, reason %s:", bag_path.c_str(),
             exception.what());
      return sdf;
    }
    printf("Constructing SDF...\n");
    populateSDFTableFromBagFile(sdf, bag);
    sdf.normalizeWeights();

    sdf.GetDistanceDebugImage().normalize(0, 255).save_bmp(("sdf_" + entry.path().filename().generic_string() + ".bmp").c_str());

    stsdfs.push_back(sdf);
  }

  SDFTable longTermSDF = SDFTable(1000, 1000, 0.1);
  for (size_t i = 0; i < stsdfs.size(); i++) {
    longTermSDF.updateWithSDF(stsdfs[i]);
  }
  longTermSDF.normalizeWeights();

  cimg_library::CImgDisplay display1;
  display1.display(longTermSDF.GetDistanceDebugImage());
  cimg_library::CImgDisplay display2;
  display2.display(longTermSDF.GetWeightDebugImage());
  longTermSDF.GetDistanceDebugImage().normalize(0, 255).save_bmp("long_term_sdf.bmp");
  longTermSDF.GetWeightDebugImage().normalize(0, 255).save_bmp("long_term_weights.bmp");
  return longTermSDF;
}

void filter_short_term_features(string bag_path) {
  SDFTable sdf = SDFTable(1000, 1000, 0.1);

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
  sdf.normalizeWeights();
  cimg_library::CImgDisplay display1;
  display1.display(sdf.GetDistanceDebugImage());

  rosbag::Bag writeBag;
  writeBag.open(bag_path, rosbag::bagmode::Append);

  printf("Filtering point clouds...\n");
  vector<string> topics;
  topics.emplace_back(FLAGS_lidar_topic.c_str());
  topics.emplace_back(FLAGS_loc_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<std::vector<Vector2f>> filteredClouds;
  Vector2f lastLoc(0, 0);
  Eigen::Rotation2Df lastOrientation(0);

  int num = 0;

  // Iterate through the bag, constructing filtered scans
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    ++num;
    const rosbag::MessageInstance& message = *it;
    {
      sensor_msgs::LaserScanPtr laser_scan =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        // Process the laser scan

        sensor_msgs::LaserScanPtr filtered(new sensor_msgs::LaserScan);
        sdf.filterScan(*laser_scan, filtered, true, lastLoc, lastOrientation);
        filtered->header.stamp = laser_scan->header.stamp;
        writeBag.write("/filtered", filtered->header.stamp, filtered);
      }

      geometry_msgs::PosePtr pose = message.instantiate<geometry_msgs::Pose>();
      if (pose != nullptr) {
        // TODO general poses
        break;
      }

      cobot_msgs::CobotLocalizationMsgPtr loc =
          message.instantiate<cobot_msgs::CobotLocalizationMsg>();
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
  } else if (FLAGS_bag_folder.compare("") != 0) {
    construct_ltsdf(FLAGS_bag_folder);
  } else {
    std::cout << "Must specify bag file, lidar topic" << std::endl;
    exit(0);
  }
  return 0;
}
