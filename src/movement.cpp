#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

enum class MoveState { IDLE, ROTATE_TO_GOAL, MOVE_TO_GOAL, ROTATE_TO_FINAL, DONE };

class MovementNode : public rclcpp::Node
{
public:
    MovementNode() : Node("movement_node")
    {
        // ---- parameter, tinggal tuning dari sini / lewat yaml ----
        declare_parameter("max_linear_speed", 0.3);
        declare_parameter("max_angular_speed", 1.0);
        declare_parameter("kp_linear", 0.6);
        declare_parameter("kp_angular", 1.8);
        declare_parameter("distance_tolerance", 0.03);   // meter
        declare_parameter("yaw_tolerance", 0.05);        // rad (~3 deg)
        declare_parameter("rotate_start_threshold", 0.4);// rad, kalau heading error > ini, putar dulu murni
        declare_parameter("control_hz", 20.0);

        max_lin_   = get_parameter("max_linear_speed").as_double();
        max_ang_   = get_parameter("max_angular_speed").as_double();
        kp_lin_    = get_parameter("kp_linear").as_double();
        kp_ang_    = get_parameter("kp_angular").as_double();
        dist_tol_  = get_parameter("distance_tolerance").as_double();
        yaw_tol_   = get_parameter("yaw_tolerance").as_double();
        rotate_thresh_ = get_parameter("rotate_start_threshold").as_double();

        double hz = get_parameter("control_hz").as_double();

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10, std::bind(&MovementNode::poseCb, this, _1));

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::QoS(10).best_effort(), std::bind(&MovementNode::goalCb, this, _1));

        cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        reached_pub_ = create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / hz),
            std::bind(&MovementNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "movement_node siap. Nunggu /goal_pose...");
    }

private:
    // ---------------- callbacks ----------------
    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        cur_x_ = msg->pose.position.x;
        cur_y_ = msg->pose.position.y;
        cur_yaw_ = yawFromQuat(msg->pose.orientation);
        has_pose_ = true;
    }

    void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_yaw_ = yawFromQuat(msg->pose.orientation);
        has_goal_ = true;
        state_ = MoveState::ROTATE_TO_GOAL;
        RCLCPP_INFO(get_logger(), "Goal baru diterima: x=%.2f y=%.2f yaw=%.2f",
                    goal_x_, goal_y_, goal_yaw_);
    }

    // ---------------- helper ----------------
    static double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tq(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
        return yaw;
    }

    static double normalizeAngle(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    void publishCmd(double lin, double ang)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(lin, -max_lin_, max_lin_);
        cmd.angular.z = std::clamp(ang, -max_ang_, max_ang_);
        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        publishCmd(0.0, 0.0);
    }

    // ---------------- main loop ----------------
    void controlLoop()
    {
        if (!has_pose_ || !has_goal_) return;
        if (state_ == MoveState::IDLE || state_ == MoveState::DONE) return;

        double dx = goal_x_ - cur_x_;
        double dy = goal_y_ - cur_y_;
        double distance = std::hypot(dx, dy);
        double angle_to_goal = std::atan2(-dx, dy);
        double heading_error = normalizeAngle(angle_to_goal - cur_yaw_);

        switch (state_)
        {
        case MoveState::ROTATE_TO_GOAL:
        {
            // kalau jaraknya udah deket banget dari awal, skip lurusin heading, langsung ke final rotate
            if (distance < dist_tol_) {
                state_ = MoveState::ROTATE_TO_FINAL;
                break;
            }
            if (std::fabs(heading_error) > yaw_tol_) {
                publishCmd(0.0, kp_ang_ * heading_error);
            } else {
                stopRobot();
                state_ = MoveState::MOVE_TO_GOAL;
                RCLCPP_INFO(get_logger(), "Heading oke, mulai maju ke goal.");
            }
            break;
        }

        case MoveState::MOVE_TO_GOAL:
        {
            if (distance < dist_tol_) {
                stopRobot();
                state_ = MoveState::ROTATE_TO_FINAL;
                RCLCPP_INFO(get_logger(), "Posisi sampai, align ke yaw akhir.");
                break;
            }

            // kalau heading error kebesaran (misal ke-overshoot / goal geser), stop dulu & putar ulang
            if (std::fabs(heading_error) > rotate_thresh_) {
                stopRobot();
                state_ = MoveState::ROTATE_TO_GOAL;
                break;
            }

            // cos(heading_error) bikin robot otomatis mundur kalau goal ada di belakang
            // (heading_error mendekati +-pi -> cos negatif -> linear jadi negatif)
            double linear = kp_lin_ * distance * std::cos(heading_error);
            double angular = kp_ang_ * heading_error;
            publishCmd(linear, angular);
            break;
        }

        case MoveState::ROTATE_TO_FINAL:
        {
            double yaw_error = normalizeAngle(goal_yaw_ - cur_yaw_);
            if (std::fabs(yaw_error) > yaw_tol_) {
                publishCmd(0.0, kp_ang_ * yaw_error);
            } else {
                stopRobot();
                state_ = MoveState::DONE;
                std_msgs::msg::Bool msg;
                msg.data = true;
                reached_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "Goal reached.");
            }
            break;
        }

        default:
            break;
        }
    }

    // ---------------- members ----------------
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    MoveState state_ = MoveState::IDLE;
    bool has_pose_ = false;
    bool has_goal_ = false;

    double cur_x_ = 0, cur_y_ = 0, cur_yaw_ = 0;
    double goal_x_ = 0, goal_y_ = 0, goal_yaw_ = 0;

    double max_lin_, max_ang_, kp_lin_, kp_ang_, dist_tol_, yaw_tol_, rotate_thresh_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementNode>());
    rclcpp::shutdown();
    return 0;
}