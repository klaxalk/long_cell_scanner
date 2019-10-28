// some ros includes
#include <ros/package.h>
#include <ros/ros.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

// Image message type is defined by the rospix node
#include <rospix/Image.h>
#include <std_msgs/Float64.h>
#include <long_cell_scanner/SetString.h>
#include <gclib_ros/Position.h>

#include <std_msgs/Float64.h>

#include <mutex>

using namespace std;

// subscriber for camera images
ros::Subscriber    image_subscriber;
ros::ServiceServer service_server_directory;

ros::Subscriber subscriber_position;
ros::Subscriber subscriber_fov;
ros::Subscriber subscriber_overlap;

// stores the name of the directory to save files
string root_directory;
string sub_directory;
string sub_sub_directory;

std::mutex mutex_sub_sub_directory;

gclib_ros::Position position;
std::mutex          mutex_position;

double fov;
std::mutex          mutex_fov;

double overlap;
std::mutex          mutex_overlap;

bool got_position = false;
bool got_fov      = false;
bool got_overlap  = false;

// is called every time new image comes in
void imageCallback(const rospix::ImageConstPtr& msg) {

  if (!got_fov || !got_overlap || !got_position) {
    return;
  }

  double date = ros::Time::now().toSec();

  // create a filename based on current time, precision on miliseconds should be enough
  char filename[40];
  sprintf(filename, "%.3f.image.txt", date);

  // add the directory name
  std::scoped_lock lock(mutex_sub_sub_directory);

  string sub_path        = string(root_directory + "/" + sub_directory);
  string final_directory = string(root_directory + "/" + sub_directory + "/" + sub_sub_directory);
  string path            = string(final_directory + "/" + string(filename));

  // tell us about saving the image
  ROS_INFO("Saving image to %s", path.c_str());

  struct stat st = {0};

  if (stat(sub_path.c_str(), &st) == -1) {
    if (mkdir(sub_path.c_str(), 0755) != 0) {
      ROS_INFO("[%s]: sub folder created", ros::this_node::getName().c_str());
    }
  }

  if (stat(final_directory.c_str(), &st) == -1) {
    if (mkdir(final_directory.c_str(), 0755) != 0) {
      ROS_INFO("[%s]: sub sub folder created", ros::this_node::getName().c_str());
    }
  }

  // open the file
  FILE* f = fopen(path.c_str(), "w");

  if (f == NULL) {
    ROS_ERROR("Cannot open the file %s for writing.", path.c_str());
  } else {

    // iterate over all pixels of the image
    for (int i = 0; i < 256; i++) {
      for (int j = 0; j < 256; j++) {

        if (msg->image[i * 256 + j] > 0) {

          // print the value
          fprintf(f, "%d %d %d\n", i, j, msg->image[i * 256 + j]);
        }
      }
    }

    // probably not neccessary, but to be sure...
    fflush(f);

    fclose(f);
  }

  // metadata file name
  sprintf(filename, "%.3f.metadata.txt", date);
  path = string(root_directory + "/" + sub_directory + "/" + sub_sub_directory + "/" + string(filename));

  // open metadata file
  f = fopen(path.c_str(), "w");

  if (f == NULL) {
    ROS_ERROR("Cannot open the file %s for writing.", path.c_str());
  } else {

    std::scoped_lock lock(mutex_position, mutex_fov, mutex_overlap);

    // print all metadata
    fprintf(f, "time: %f\n", msg->stamp.toSec());
    fprintf(f, "exposure_time: %f\n", msg->exposure_time);
    fprintf(f, "mode: %s\n", msg->mode == 0 ? "MPX" : "TOT");
    fprintf(f, "threshold: %d\n", msg->threshold);
    fprintf(f, "bias: %f\n", msg->bias);
    fprintf(f, "interface: %s\n", msg->interface.c_str());
    fprintf(f, "clock: %d\n", msg->clock);
    fprintf(f, "position x: %.4f\n", position.position[0]);
    fprintf(f, "position y: %.4f\n", position.position[1]);
    fprintf(f, "fov: %.4f\n", fov);
    fprintf(f, "overlap: %.4f\n", overlap);

    // probably not neccessary, but to be sure...
    fflush(f);

    fclose(f);
  }
}

// is called every time new image comes in
bool callbackSetDirectory(long_cell_scanner::SetString::Request& req, long_cell_scanner::SetString::Response& res) {

  std::scoped_lock lock(mutex_sub_sub_directory);

  sub_sub_directory = req.value;

  res.success = true;
  res.message = "directory set";

  return true;
}

void callbackPosition(const gclib_ros::PositionConstPtr& msg) {

  std::scoped_lock lock(mutex_position);

  position = *msg;

  got_position = true;
}

void callbackFov(const std_msgs::Float64ConstPtr& msg) {

  std::scoped_lock lock(mutex_fov);

  fov = msg->data;

  got_fov = true;
}

void callbackOverlap(const std_msgs::Float64ConstPtr& msg) {

  std::scoped_lock lock(mutex_overlap);

  overlap = msg->data;

  got_overlap = true;
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "saver");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // load parameters from config file (launch file)
  nh_.param("root_directory", root_directory, string());
  nh_.param("sub_directory", sub_directory, string());
  nh_.param("sub_sub_directory", sub_sub_directory, string());

  // SUBSCRIBERS
  image_subscriber         = nh_.subscribe("image_in", 1, &imageCallback, ros::TransportHints().tcpNoDelay());
  service_server_directory = nh_.advertiseService("set_directory", &callbackSetDirectory);

  subscriber_position = nh_.subscribe("position_in", 1, &callbackPosition, ros::TransportHints().tcpNoDelay());
  subscriber_fov      = nh_.subscribe("fov_in", 1, &callbackFov, ros::TransportHints().tcpNoDelay());
  subscriber_overlap  = nh_.subscribe("overlap_in", 1, &callbackOverlap, ros::TransportHints().tcpNoDelay());

  // needed to make stuff work
  ros::spin();

  return 0;
}
