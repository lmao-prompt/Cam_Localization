#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

struct Waypoint {
    double x, y;
};

class WaypointNav : public rclcpp::Node
{
public:
    WaypointNav() : Node("waypoint_nav"),
        current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
        wp_index_(0), has_pose_(false)
    {
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10,
            std::bind(&WaypointNav::poseCb, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Daftar waypoint (meter, world frame)
        waypoints_ = {
            {0.3,  0.0},
            {0.3,  0.3},
            {0.0,  0.3},
            {0.0,  0.0},
        };

        RCLCPP_INFO(get_logger(), "Waypoint Nav started, %zu waypoints", waypoints_.size());
    }

private:
    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        has_pose_ = true;

        navigate();
    }

    void navigate()
    {
        if (!has_pose_ || wp_index_ >= waypoints_.size()) {
            stop();
            if (wp_index_ >= waypoints_.size())
                RCLCPP_INFO_ONCE(get_logger(), "All waypoints reached!");
            return;
        }

        Waypoint &wp = waypoints_[wp_index_];

        double dx = wp.x - current_x_;
        double dy = wp.y - current_y_;
        double dist = std::hypot(dx, dy);

        // Sudah sampai waypoint ini?
        if (dist < dist_tol_) {
            RCLCPP_INFO(get_logger(), "Reached waypoint %zu (%.2f, %.2f)", wp_index_, wp.x, wp.y);
            wp_index_++;
            return;
        }

        // Hitung heading error
        double target_yaw = std::atan2(dy, dx);
        double yaw_err = target_yaw - current_yaw_;

        // Normalize ke [-pi, pi]
        while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;

        if (std::fabs(yaw_err) > yaw_tol_) {
            // Rotate dulu sebelum maju
            cmd.linear.x  = 0.0;
            cmd.angular.z = kp_yaw_ * yaw_err;
            cmd.angular.z = std::clamp(cmd.angular.z, -max_ang_, max_ang_);
        } else {
            // Maju ke waypoint
            cmd.linear.x  = kp_lin_ * dist;
            cmd.linear.x  = std::clamp(cmd.linear.x, 0.0, max_lin_);
            cmd.angular.z = kp_yaw_ * yaw_err; // koreksi heading kecil
        }

        pub_cmd_->publish(cmd);

        RCLCPP_INFO(get_logger(),
            "WP[%zu] dist=%.2f yaw_err=%.2f | vx=%.2f wz=%.2f",
            wp_index_, dist, yaw_err * 180.0 / M_PI,
            cmd.linear.x, cmd.angular.z);
    }

    void stop()
    {
        geometry_msgs::msg::Twist cmd;
        pub_cmd_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    std::vector<Waypoint> waypoints_;
    size_t wp_index_;

    double current_x_, current_y_, current_yaw_;
    bool has_pose_;

    // Tuning parameter
    const double kp_lin_  = 0.5;   // gain linear
    const double kp_yaw_  = 1.0;   // gain angular
    const double max_lin_ = 0.3;   // max linear velocity (m/s)
    const double max_ang_ = 0.8;   // max angular velocity (rad/s)
    const double dist_tol_ = 0.15; // toleransi jarak (m)
    const double yaw_tol_  = 0.1;  // toleransi heading (rad)
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNav>());
    rclcpp::shutdown();
    return 0;
}