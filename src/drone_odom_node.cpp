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

using namespace std::chrono_literals;


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

        this->timer_ = this->create_wall_timer(5ms, std::bind(&DroneOdomNode::odom_publisher_timer_callback, this));

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
        // this->ox.normalize();
        // this->oy.normalize();
        // this->oz = this->oy.cross(this->ox);
        this->oz = this->ox.cross(this->oy);
        // this->oz.normalize();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        // odom_msg.child_frame_id = "base_link";

        // find the centroid by averaging Ox and Oy
        odom_msg.pose.pose.position.x = (this->Ox.x() + this->Oy.x()) / 2.0;
        odom_msg.pose.pose.position.y = (this->Ox.y() + this->Oy.y()) / 2.0;
        odom_msg.pose.pose.position.z = (this->Ox.z() + this->Oy.z()) / 2.0;


        // create rotation matrix from ox oy and oz
        this->rot_mat.row(0) = this->ox.normalized();
        this->rot_mat.row(1) = this->oy.normalized();
        this->rot_mat.row(2) = this->oz.normalized();

        // convert rotation matrix to quaternion,

        double angle_rad = M_PI; // 180 degrees in radians

        // Create a rotation matrix for rotation about the x-axis
        Eigen::Matrix3d correction_matrix;
        correction_matrix = Eigen::AngleAxisd(-angle_rad/2.0, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d correction_matrix1;
        correction_matrix1 = Eigen::AngleAxisd(-angle_rad/2.0, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d correction_matrix2;
        correction_matrix2 = Eigen::AngleAxisd(-angle_rad/2.0, Eigen::Vector3d::UnitY());
        // correction_matrix1 <<
        //     1, 0, 0,
        //     0, 1, 0,
        //     0, 0, -1;


        // Eigen::Matrix3d reflection_matrix;
        // reflection_matrix <<
        //     -1, 0, 0,
        //     0, 1, 0,
        //     0, 0, 1;

        // this->rot_mat = this->rot_mat * correction_matrix * correction_matrix1 * correction_matrix2 * correction_matrix1;

        this->orientation = this->rot_mat;
        // this->orientation = this->orientation * Eigen::Quaternion<double>(correction_matrix);
        // this->orientation = Eigen::Quaternion<double>(correction_matrix) * this->orientation;
        // this->orientation = this->orientation * Eigen::Quaternion<double>(correction_matrix1);
        // this->orientation = this->orientation * Eigen::Quaternion<double>(reflection_matrix);
        // double q0 = sqrt(1 + rot_mat(0, 0) + rot_mat(1, 1) + rot_mat(2, 2)) / 2.0;
        // this->orientation.w() = q0;
        // this->orientation.x() = (rot_mat(2, 1) - rot_mat(1, 2)) / (4.0 * q0);
        // this->orientation.y() = (rot_mat(0, 2) - rot_mat(2, 0)) / (4.0 * q0);
        // this->orientation.z() = (rot_mat(1, 0) - rot_mat(0, 1)) / (4.0 * q0);
        this->orientation.normalize();

        odom_msg.pose.pose.orientation.w = this->orientation.w();
        odom_msg.pose.pose.orientation.x = this->orientation.x();
        odom_msg.pose.pose.orientation.y = this->orientation.y();
        odom_msg.pose.pose.orientation.z = this->orientation.z();
        // odom_msg.pose.pose.orientation.w = this->orientation.w();
        // odom_msg.pose.pose.orientation.x = this->orientation.x();
        // odom_msg.pose.pose.orientation.y = this->orientation.y();
        // odom_msg.pose.pose.orientation.z = this->orientation.z();

        // publish
        this->odom_publisher->publish(odom_msg);
    }

    void mocap_markers_callback(const MarkersMsg::SharedPtr msg) {
        std::vector<marker_t> markers = msg->markers;
        int found = 0;
        for (int u = 0; u < markers.size(); u++) {
          if (markers[u].id == this->marker_o) {
              this->Oo = Eigen::Vector3d(markers[u].x, markers[u].y, markers[u].z);
              // RCLCPP_INFO(this->get_logger(), "found marker_o");
              if (markers[u].cond != -1) found++;
          } else if (markers[u].id == this->marker_x) {
              this->Ox = Eigen::Vector3d(markers[u].x, markers[u].y, markers[u].z);
              // RCLCPP_INFO(this->get_logger(), "found marker_x");
              if (markers[u].cond != -1) found++;
          } else if (markers[u].id == this->marker_y) {
              this->Oy = Eigen::Vector3d(markers[u].x, markers[u].y, markers[u].z);
              // RCLCPP_INFO(this->get_logger(), "found marker_y");
              if (markers[u].cond != -1) found++;
          } else {
            continue;
          }

          if (found == 3)
              break;
        }

        if (found != 3) {
            RCLCPP_WARN(this->get_logger(), "unable to locate all markers");
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
