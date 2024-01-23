#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "phasespace_msgs/msg/markers.hpp"
#include "phasespace_msgs/msg/marker.hpp"

#define MARKERS_TOPIC "/phasespace/markers"

#define DBG
#define SAMPLES_TO_AVG 4000


using MarkersMsg = phasespace_msgs::msg::Markers;
using marker_t = phasespace_msgs::msg::Marker;

using graph_idx_t = int;
using marker_id_t = int;

std::vector<long int> marker_ids;

std::shared_ptr<std::map<graph_idx_t, marker_id_t>> graph_to_marker_map;
std::shared_ptr<std::map<marker_id_t, graph_idx_t>> marker_to_graph_map;
std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>> tf_tree;
std::shared_ptr<std::vector<std::vector<int>>> num_tf_tree_readings;
std::shared_ptr<std::vector<std::vector<bool>>> tf_tree_reading_statuses;

bool find_in_markers(std::vector<marker_t> markers_, int id) {
  for (int i = 0; i < markers_.size(); i++)
    if (markers_[i].id == id)
      return true;

  return false;
}

bool find_in_vector(std::vector<long int> vec, int element) {
  return std::find(vec.begin(), vec.end(), element) == vec.end();
}

void markers_callback(const MarkersMsg::SharedPtr msg) {
  std::vector<marker_t> markers = msg->markers;
  for (int u = 0; u < markers.size(); u++) {
    if ((markers[u].cond == -1)) // skip if mocap system hasn't found the marker
      continue;
    else if (!find_in_vector(marker_ids, markers[u].id)) // check if the ID is in the list of markers we need, skip if the marker doesn't belong to the body we are interested in capturing
      continue;

    // find the graph_idx of the id in marker[u]
    int ind_x = (*marker_to_graph_map)[markers[u].id];

    for (int v=0; v < markers.size(); v++) {
      if ((u == v) || // if both are same the vector will be (0, 0, 0)
          (markers[v].cond == -1)) // skip if the mocap hasn't found the marker
        continue;
      else if (!find_in_vector(marker_ids, markers[u].id)) // skip if the marker doesn't belong to the body we are interested in capturing
        continue;

      int ind_y = (*marker_to_graph_map)[markers[v].id];
      if ((*num_tf_tree_readings)[ind_x][ind_y] >=
          SAMPLES_TO_AVG) { // skip if enough samples are collected
        if (!(*tf_tree_reading_statuses)[ind_x][ind_y]) // but before skipping just check if it has been averaged or not, if not average it
          (*tf_tree)[ind_x][ind_y] /= SAMPLES_TO_AVG;

        continue;
      }

      // store the transform between ind_x -> ind_y
      Eigen::Vector3d x_to_y = Eigen::Vector3d(markers[ind_y].x - markers[ind_x].x,
                                               markers[ind_y].y - markers[ind_x].y,
                                               markers[ind_y].z - markers[ind_x].z);
      (*tf_tree)[ind_x][ind_y] += x_to_y;
      (*num_tf_tree_readings)[ind_x][ind_y]++;
    }
  }
}

int main(int argc, char ** argv) {
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto rb_capture_node = std::make_shared<rclcpp::Node>("rb_capture_node");
  rb_capture_node->declare_parameter("marker_ids", rclcpp::PARAMETER_INTEGER_ARRAY);
  rb_capture_node->declare_parameter("drone_name", rclcpp::PARAMETER_STRING);

  rclcpp::Parameter marker_ids_param = rb_capture_node->get_parameter("marker_ids");
  rclcpp::Parameter drone_name_param = rb_capture_node->get_parameter("drone_name");
  std::string drone_name = drone_name_param.as_string();
  marker_ids = marker_ids_param.as_integer_array();
  int num_markers = marker_ids.size();
  if (num_markers != 8) {
    RCLCPP_WARN(rb_capture_node->get_logger(), "8 marker IDs haven't been provided, a higher number of markers makes it easier to localize the body\n");
  }

  // initialize the tree_id_table
  graph_to_marker_map = std::make_shared<std::map<graph_idx_t, marker_id_t>>();
  marker_to_graph_map = std::make_shared<std::map<marker_id_t, graph_idx_t>>();
  for (int i=0; i < num_markers; i++) {
    (*graph_to_marker_map)[i] = marker_ids[i];
    (*marker_to_graph_map)[marker_ids[i]] = i;
  }

  // allocate memory to the tf tree
  tf_tree = std::make_shared<std::vector<std::vector<Eigen::Vector3d>>>(num_markers, std::vector<Eigen::Vector3d>(num_markers, Eigen::Vector3d(0, 0, 0)));
  num_tf_tree_readings = std::make_shared<std::vector<std::vector<int>>>(num_markers, std::vector<int>(num_markers));
  tf_tree_reading_statuses = std::make_shared<std::vector<std::vector<bool>>>(num_markers, std::vector<bool>(num_markers, false));

  rclcpp::Subscription<MarkersMsg>::SharedPtr markers_subscription =
    rb_capture_node->create_subscription<MarkersMsg>(MARKERS_TOPIC, 10, &markers_callback);


  rclcpp::spin(rb_capture_node);
  return 0;
}
