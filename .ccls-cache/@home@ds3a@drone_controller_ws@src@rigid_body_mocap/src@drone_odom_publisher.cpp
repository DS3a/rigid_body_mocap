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
#include "phasespace_msgs/msg/rigids.hpp"
#include "phasespace_msgs/msg/rigid.hpp"

#define MARKERS_TOPIC "/phasespace/markers"
#define RIGIDS_TOPIC "/phasespace/rigids"

using MarkersMsg = phasespace_msgs::msg::Markers;
using marker_t = phasespace_msgs::msg::Marker;

using RigidsMsg = phasespace_msgs::msg::Rigids;
using rigid_t = phasespace_msgs::msg::Rigid;

using namespace std::chrono_literals;

class DroneOdomNode : public rclcpp::Node {
    public:
    DroneOdomNode() : Node("drone_odom_node") {
        this->declare_parameter("drone_id", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("odom_rate", rclcpp::PARAMETER_DOUBLE);

        rclcpp::Parameter drone_name_param = this->get_parameter("drone_id");
        rclcpp::Parameter odom_rate_param = this->get_parameter("odom_rate");

        this->drone_name = drone_name_param.as_int();

        this->odom_rate = odom_rate_param.as_double();

        this->mocap_rigids_subscription = this->create_subscription<RigidsMsg>(RIGIDS_TOPIC, 10,
                                                                                 std::bind(&DroneOdomNode::mocap_rigids_callback,
                                                                                           this,
                                                                                           std::placeholders::_1));

        this->odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        this->timer_ = this->create_wall_timer(5ms, std::bind(&DroneOdomNode::odom_publisher_timer_callback, this));

    }

    private:

    // ROS shit
    rclcpp::Subscription<RigidsMsg>::SharedPtr mocap_rigids_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;


    // other shit
    double odom_rate;
    int drone_name;

    Eigen::Vector3d position;
    Eigen::Quaternion<double> orientation;
    bool drone_in_view=false;

    void odom_publisher_timer_callback() {
        if (!this->drone_in_view)
            return;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "drone_base_link";

        odom_msg.pose.pose.position.x = this->position.x();
        odom_msg.pose.pose.position.x = this->position.x();
        odom_msg.pose.pose.position.x = this->position.x();

        Eigen::Matrix3d rot_mat;
        rot_mat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        this->orientation *= Eigen::Quaternion<double>(rot_mat);

        this->orientation.normalize();
        odom_msg.pose.pose.orientation.w = this->orientation.normalized().w();
        odom_msg.pose.pose.orientation.x = this->orientation.normalized().x();
        odom_msg.pose.pose.orientation.y = this->orientation.normalized().y();
        odom_msg.pose.pose.orientation.z = this->orientation.normalized().z();

        this->odom_publisher->publish(odom_msg);
    }

    void mocap_rigids_callback(const RigidsMsg::SharedPtr msg) {
        std::vector<rigid_t> rigids = msg->rigids;
        int found = 0;
        for (int u = 0; u < rigids.size(); u++) {
            if (rigids[u].id == this->drone_name) {
                this->orientation.w() = rigids[u].qw;
                this->orientation.x() = rigids[u].qx;
                this->orientation.y() = rigids[u].qy;
                this->orientation.z() = rigids[u].qz;

                this->position.x() = rigids[u].x;
                this->position.y() = rigids[u].y;
                this->position.z() = rigids[u].z;

                found = 1;
                break;
            }
        }

        if (found != 1) {
            RCLCPP_WARN(this->get_logger(), "unable to locate the rigid marker");
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
