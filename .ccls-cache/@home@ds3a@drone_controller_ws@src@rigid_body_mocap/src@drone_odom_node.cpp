#include <stdio.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>

#include "nav_msgs/msg/odometry.hpp"
#include "phasespace_msgs/msg/markers.hpp"
#include "phasespace_msgs/msg/marker.hpp"

#define MARKERS_TOPIC "/phasespace/markers"

using MarkersMsg = phasespace_msgs::msg::Markers;
using marker_t = phasespace_msgs::msg::Marker;

class DroneOdomNode : public rclcpp::Node {
    public:
    DroneOdomNode() : Node("drone_odom_node") {
        this->declare_parameter("drone_name", rclcpp::PARAMETER_STRING);
        this->declare_parameter("marker_o", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("marker_x", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("marker_y", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("odom_rate", rclcpp::PARAMETER_DOUBLE);

        rclcpp::Parameter drone_name_param = this->get_parameter("drone_name");
        rclcpp::Parameter marker_o_param = this->get_parameter("marker_o");
        rclcpp::Parameter marker_x_param = this->get_parameter("marker_x");
        rclcpp::Parameter marker_y_param = this->get_parameter("marker_y");
        rclcpp::Parameter odom_rate_param = this->get_parameter("odom_rate");

        this->drone_name = drone_name_param.as_string();

        this->marker_o = marker_o_param.as_int();
        this->marker_x = marker_x_param.as_int();
        this->marker_y = marker_y_param.as_int();
        this->odom_rate = odom_rate_param.as_double();

        this->mocap_markers_subscription = this->create_subscription<MarkersMsg>(MARKERS_TOPIC, 10,
                                                                                 std::bind(&DroneOdomNode::mocap_markers_callback,
                                                                                           this,
                                                                                           std::placeholders::_1));
        this->odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // this->timer_ = this->create_create_wall_timer(500ms, );

    }

    private:

    // ROS shit
    rclcpp::Subscription<MarkersMsg>::SharedPtr mocap_markers_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;


    // other shit
    double odom_rate;
    int marker_o;
    int marker_x;
    int marker_y;
    std::string drone_name;
    Eigen::Quaternion<double> orientation;
    Eigen::Matrix3d rot_mat;
    Eigen::Vector3d Oo;
    Eigen::Vector3d Ox;
    Eigen::Vector3d Oy;

    Eigen::Vector3d ox;
    Eigen::Vector3d oy;
    Eigen::Vector3d oz;
    bool drone_in_view=true;

    void odom_publisher_timer_callback() {
        if (!this->drone_in_view)
            return;

        this->ox = this->Ox - this->Oo;
        this->oy = this->Oy - this->Oo;
        this->ox.normalize();
        this->oy.normalize();
        this->oz = this->ox.cross(this->oy);


        nav_msgs::msg::Odometry odom_msg;
        odom_msg.pose.pose.position.x = (this->Ox.x() + this->Oy.x()) / 2.0;
        odom_msg.pose.pose.position.y = (this->Ox.y() + this->Oy.y()) / 2.0;
        odom_msg.pose.pose.position.z = (this->Ox.z() + this->Oy.z()) / 2.0;

        // TODO create rotation matrix from ox oy and oz
        // TODO convert rotation matrix to quaternion, and find the centroid by averaging Ox and Oy
        // publish
    }

    void mocap_markers_callback(const MarkersMsg::SharedPtr msg) {
        std::vector<marker_t> markers = msg->markers;
        int found = 0;
        for (int u = 0; u < markers.size(); u++) {
          if (markers[u].id == this->marker_o) {
          } else if (markers[u].id == this->marker_o) {
              this->Oo = Eigen::Vector3d(markers[u].x, markers[u].y, markers[u].z);
              if (markers[u].cond != -1) found++;
          } else if (markers[u].id == this->marker_x) {
              this->Ox = Eigen::Vector3d(markers[u].x, markers[u].y, markers[u].z);
              if (markers[u].cond != -1) found++;
          } else if (markers[u].id == this->marker_y) {
              this->Oy = Eigen::Vector3d(markers[u].x, markers[u].y, markers[u].z);
              if (markers[u].cond != -1) found++;
          } else {
            continue;
          }
        }

        if (found != 3) {
            this->drone_in_view = false;
        } else {
            this->drone_in_view = true;
        }
    }
};


int main(int argc, char ** argv) {
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneOdomNode>());
    rclcpp::shutdown();

    return 0;
}
