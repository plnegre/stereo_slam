#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "constants.h"
#include "publisher.h"
#include "tracking.h"
#include "graph.h"
#include "loop_closing.h"

namespace fs = boost::filesystem;

/** \brief Read the node parameters
  */
void readTrackingParams(slam::Tracking::Params &tracking_params)
{
  ros::NodeHandle nhp("~");
  nhp.param("odom_topic",   tracking_params.odom_topic,   string(""));
  nhp.param("camera_topic", tracking_params.camera_topic, string(""));
  nhp.param("refine",       tracking_params.refine,       false);
}

void recursiveCopy(const fs::path &src, const fs::path &dst)
{
  if (fs::exists(dst))
  {
    throw std::runtime_error(dst.generic_string() + " exists");
  }

  if (fs::is_directory(src))
  {
    fs::create_directories(dst);
    for (fs::directory_entry& item : fs::directory_iterator(src)) {
      recursiveCopy(item.path(), dst/item.path().filename());
    }
  }
  else if (fs::is_regular_file(src)) {
    fs::copy(src, dst);
  }
  else {
    throw std::runtime_error(dst.generic_string() + " not dir or file");
  }
}

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "stereo_slam");
  ros::start();

  // Create the output directory
  string output_dir = slam::WORKING_DIRECTORY;
  if (fs::is_directory(output_dir))
  {
    // Make a backup
    int counter = 1;
    bool exists = true;
    string tmp_dir;
    while (exists) {
      tmp_dir = output_dir.substr(0, output_dir.size()-1) + "_bk/" + to_string(counter);

      if (fs::is_directory(tmp_dir))
        counter++;
      else
        exists= false;
    }
    recursiveCopy(output_dir, tmp_dir);

    ROS_WARN_STREAM("[Localization:] WARNING -> The output directory " <<
      "already exists. Creating a backup of " << output_dir << " into " <<
      tmp_dir);

    // And remove
    fs::remove_all(output_dir);
  }
  fs::path dir0(output_dir);
  if (!fs::create_directory(dir0))
    ROS_ERROR("[Localization:] ERROR -> Impossible to create the output directory.");

  // For debugging purposes
  slam::Publisher publisher;

  // Threads
  slam::LoopClosing loop_closing;
  slam::Graph graph(&loop_closing);
  slam::Tracking tracker(&publisher, &graph);

  // Read parameters
  slam::Tracking::Params tracking_params;
  readTrackingParams(tracking_params);

  // Set the parameters for every object
  tracker.setParams(tracking_params);
  loop_closing.setGraph(&graph);

  // Launch threads
  boost::thread trackingThread(&slam::Tracking::run, &tracker);
  boost::thread graphThread(&slam::Graph::run, &graph);
  boost::thread loopClosingThread(&slam::LoopClosing::run, &loop_closing);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
  }

  // Loop closing object is the only one that needs finalization
  loop_closing.finalize();

  ros::shutdown();

  return 0;
}