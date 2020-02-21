#include <csignal>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>

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

using std::string;
using std::vector;
using namespace Eigen;

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

// Given 2 scans calculate relative transformation & uncertainty
void filter_short_term_features(string bag_path, string lidar_topic, string loc_topic) {
  printf("Loading bag file... ");
  std::vector<Vector2f> baseCloud;
  std::vector<Vector2f> matchCloud;

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& exception) {
    printf("Unable to read %s, reason %s:", bag_path.c_str(), exception.what());
    return;
  }
  // Get the topics we want
  vector<string> topics;
  topics.emplace_back(lidar_topic.c_str());
  topics.emplace_back(loc_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  printf("Bag file has %d scans\n", view.size());

  SDFTable table = SDFTable(10, 0.2);
  SDFTable global = SDFTable(400, 400, 0.25);

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
        // table.populateFromScan(*laser_scan, true);
        global.populateFromScan(*laser_scan, true, lastLoc, lastOrientation);
        // break;
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
  bag.close();
  printf("Done.\n");
  fflush(stdout);
 
  // -39, -21 -> 1.8, 21.8

  cimg_library::CImgDisplay display1;
  display1.display(global.GetDistanceDebugImage().resize_doubleXY().normalize());
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
    filter_short_term_features(FLAGS_bag_file, FLAGS_lidar_topic, FLAGS_loc_topic);
  } else {
    std::cout << "Must specify bag file, lidar topic" << std::endl;
    exit(0);
  }
  return 0;
}
