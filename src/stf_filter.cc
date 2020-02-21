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
#include "pointcloud_helpers.h"

using std::string;
using std::vector;
using sensor_msgs::PointCloud2;
using namespace Eigen;

DEFINE_string(
  bag_file,
  "",
  "Bag file from which to read scans. Only works if scan_match_topic isn't provided.");
DEFINE_string(
  lidar_topic,
  "/Cobot/Laser",
  "topic within bag file which to read scans. Only works if scan_match_topic isn't provided.");
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
void filter_short_term_features(string bag_path, string lidar_topic) {
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
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  printf("Bag file has %d scans\n", view.size());
  // Iterate through the bag
  for (rosbag::View::iterator it = view.begin();
       it != view.end();
       ++it) {
    const rosbag::MessageInstance &message = *it;
    {
      // Load all the point clouds into memory.
      sensor_msgs::LaserScanPtr laser_scan =
              message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        
        // Process the laser scan
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);

}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0) {
    filter_short_term_features(FLAGS_bag_file, FLAGS_lidar_topic);
  } else {
    std::cout << "Must specify bag file, lidar topic" << std::endl;
    exit(0);
  }
  return 0;
}
