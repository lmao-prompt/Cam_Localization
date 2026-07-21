#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// ================== PID CONTROLLER ==================
class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double out_min, double out_max)
        : kp_(kp), ki_(ki), kd_(kd),
          out_min_(out_min), out_max_(out_max),
          integral_(0.0), prev_error_(0.0), first_run_(true)
    {
    }

    double compute(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;

        double p_term = kp_ * error;

        integral_ += error * dt;
        double i_max = out_max_ / std::max(ki_, 1e-9);
        integral_ = std::clamp(integral_, -i_max, i_max);
        double i_term = ki_ * integral_;

        double d_term = 0.0;
        if (!first_run_)
        {
            d_term = kd_ * (error - prev_error_) / dt;
        }
        prev_error_ = error;
        first_run_ = false;

        double output = p_term + i_term + d_term;
        return std::clamp(output, out_min_, out_max_);
    }

    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;
    }

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double integral_;
    double prev_error_;
    bool first_run_;
};

// ================== STATE MACHINE CHASE ==================
enum class ChaseState
{
    SEARCHING,
    ALIGNING,
    APPROACHING,
    REACHED
};

class PoseFusion : public rclcpp::Node
{
public:
    PoseFusion() : Node("pose_fusion"),
                    x_(0.0), y_(0.0), yaw_(0.0), has_pos_(false),
                    state_(ChaseState::SEARCHING),
                    yellow_detected_(false), yellow_cx_(0), yellow_area_(0),
                    pid_angular_(0, 0, 0, 0, 0),
                    pid_linear_(0, 0, 0, 0, 0)
    {
        // ---------- parameter aruco / pose ----------
        declare_parameter("robot_marker_id", 312); // sesuai ID yang keliatan di kamera kamu
        declare_parameter("aruco_yaw_offset_deg", 90.0);  // sudah dikalibrasi: robot ngadep 0deg -> raw aruco -90deg
        declare_parameter("aruco_yaw_invert", false);    // flip sign kalau arah rotasinya kebalik

        // ---------- parameter chase (yellow) ----------
        declare_parameter("image_width", 640);
        declare_parameter("target_area", 15000);
        declare_parameter("area_tolerance", 1500);
        declare_parameter("align_threshold_px", 80);
        declare_parameter("realign_threshold_px", 150); // longgar, biar gak bolak-balik ALIGNING<->APPROACHING
        declare_parameter("detection_timeout_ms", 500);
        declare_parameter("control_rate_hz", 20.0);

        declare_parameter("angular_kp", 0.01);
        declare_parameter("angular_ki", 0.0002);
        declare_parameter("angular_kd", 0.0008);
        declare_parameter("angular_max", 0.6);

        declare_parameter("linear_kp", 0.00006);
        declare_parameter("linear_ki", 0.000003);
        declare_parameter("linear_kd", 0.00001);
        declare_parameter("linear_max", 0.25);

        robot_marker_id_ = get_parameter("robot_marker_id").as_int();
        aruco_yaw_offset_ = get_parameter("aruco_yaw_offset_deg").as_double() * M_PI / 180.0;
        aruco_yaw_invert_ = get_parameter("aruco_yaw_invert").as_bool();

        image_width_ = get_parameter("image_width").as_int();
        target_area_ = get_parameter("target_area").as_int();
        area_tolerance_ = get_parameter("area_tolerance").as_int();
        align_threshold_px_ = get_parameter("align_threshold_px").as_int();
        realign_threshold_px_ = get_parameter("realign_threshold_px").as_int();
        detection_timeout_ms_ = get_parameter("detection_timeout_ms").as_int();
        double control_rate_hz = get_parameter("control_rate_hz").as_double();

        double a_kp = get_parameter("angular_kp").as_double();
        double a_ki = get_parameter("angular_ki").as_double();
        double a_kd = get_parameter("angular_kd").as_double();
        double a_max = get_parameter("angular_max").as_double();

        double l_kp = get_parameter("linear_kp").as_double();
        double l_ki = get_parameter("linear_ki").as_double();
        double l_kd = get_parameter("linear_kd").as_double();
        double l_max = get_parameter("linear_max").as_double();

        pid_angular_ = PIDController(a_kp, a_ki, a_kd, -a_max, a_max);
        pid_linear_ = PIDController(l_kp, l_ki, l_kd, 0.0, l_max); // gak mundur

        declare_parameter("homography_path", std::string("/home/prhayogo/Downloads/Nexus_Gazebo/src/homography.json"));
        std::string homography_path = get_parameter("homography_path").as_string();
        homography_ = loadHomography(homography_path);

        // ---------- subscriber / publisher pose ----------
        sub_pos_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/aruco_markers", 10,
            std::bind(&PoseFusion::arucoCb, this, std::placeholders::_1));

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10);

        // ---------- subscriber / publisher chase ----------
        sub_yellow_ = create_subscription<geometry_msgs::msg::Point>(
            "/yellow_position", 10,
            std::bind(&PoseFusion::yellowCb, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        last_yellow_time_ = now();

        auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&PoseFusion::controlLoop, this));

        RCLCPP_INFO(get_logger(),
            "Pose Fusion + Chase started. marker_id=%d, control_rate=%.1fHz, yaw_source=ARUCO",
            robot_marker_id_, control_rate_hz);
    }

private:
    // ================== ARUCO POSE + YAW ==================
    void arucoCb(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        const auto &data = msg->data;
        if (data.size() < 5) return;

        for (size_t i = 1; i + 3 < data.size(); i += 4)
        {
            int id = data[i];
            int cx = data[i + 1];
            int cy = data[i + 2];
            int angle_mdeg = data[i + 3];

            if (id == robot_marker_id_)
            {
                cv::Point2f world = pixelToWorld(cx, cy);
                x_ = world.x;
                y_ = world.y;
                has_pos_ = true;

                // theta aruco sebagai referensi yaw utama (bukan IMU)
                double yaw_aruco = (angle_mdeg / 1000.0) * M_PI / 180.0;
                if (aruco_yaw_invert_) yaw_aruco = -yaw_aruco;
                yaw_aruco += aruco_yaw_offset_;
                yaw_ = std::atan2(std::sin(yaw_aruco), std::cos(yaw_aruco)); // normalize [-pi, pi]

                publishPose();
                break;
            }
        }
    }

    cv::Mat loadHomography(const std::string &path)
    {
        std::ifstream f(path);
        if (!f.is_open())
        {
            RCLCPP_WARN(get_logger(),
                "Gak nemu %s, pakai identity matrix (x/y masih pixel, bukan meter!). "
                "Jalanin script kalibrasi dulu.", path.c_str());
            return cv::Mat::eye(3, 3, CV_64F);
        }

        nlohmann::json j;
        try
        {
            f >> j;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Gagal parse %s: %s. Pakai identity matrix.",
                path.c_str(), e.what());
            return cv::Mat::eye(3, 3, CV_64F);
        }

        auto H_data = j.at("H").get<std::vector<std::vector<double>>>();
        if (H_data.size() != 3 || H_data[0].size() != 3)
        {
            RCLCPP_ERROR(get_logger(), "Format H di %s salah (harus 3x3). Pakai identity matrix.",
                path.c_str());
            return cv::Mat::eye(3, 3, CV_64F);
        }

        cv::Mat H(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                H.at<double>(r, c) = H_data[r][c];

        RCLCPP_INFO(get_logger(), "Homography loaded dari %s. x/y sekarang dalam meter.",
            path.c_str());
        return H;
    }

    cv::Point2f pixelToWorld(int px, int py)
    {
        std::vector<cv::Point2f> src = {{(float)px, (float)py}};
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homography_);
        return dst[0];
    }

    void publishPose()
    {
        if (!has_pos_) return;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x_;
        pose.pose.position.y = y_;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pub_pose_->publish(pose);

        RCLCPP_INFO(get_logger(), "Pose -> x: %.2f  y: %.2f  yaw: %.2f deg",
            x_, y_, yaw_ * 180.0 / M_PI);
    }

    // ================== CHASE (YELLOW) ==================
    void yellowCb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // format: x=cx, y=cy, z=area. Gak ada flag "detected" eksplisit,
        // jadi tiap pesan masuk dianggap deteksi valid; lost-detection
        // ditangani lewat watchdog timeout di controlLoop().
        yellow_detected_ = true;
        yellow_cx_ = static_cast<int>(msg->x);
        yellow_area_ = static_cast<int>(msg->z);
        last_yellow_time_ = now();
    }

    void controlLoop()
    {
        double dt = 1.0 / std::max(get_parameter("control_rate_hz").as_double(), 1.0);

        auto elapsed_ms = (now() - last_yellow_time_).nanoseconds() / 1e6;
        bool timed_out = elapsed_ms > detection_timeout_ms_;

        if (!yellow_detected_ || timed_out)
        {
            if (state_ != ChaseState::SEARCHING)
            {
                RCLCPP_WARN(get_logger(), "Yellow object lost, stopping.");
            }
            state_ = ChaseState::SEARCHING;
            pid_angular_.reset();
            pid_linear_.reset();
            stopRobot();
            return;
        }

        int error_x = yellow_cx_ - (image_width_ / 2);
        int error_area = target_area_ - yellow_area_;

        // ---- state transition (hysteresis, biar gak chattering) ----
        if (std::abs(error_area) <= area_tolerance_)
        {
            state_ = ChaseState::REACHED;
        }
        else if (state_ == ChaseState::ALIGNING)
        {
            // baru boleh pindah maju kalau udah cukup lurus (threshold ketat)
            if (std::abs(error_x) <= align_threshold_px_)
            {
                state_ = ChaseState::APPROACHING;
                pid_angular_.reset();
            }
        }
        else if (state_ == ChaseState::APPROACHING || state_ == ChaseState::REACHED)
        {
            // balik align cuma kalau geser jauh banget (threshold longgar)
            if (std::abs(error_x) > realign_threshold_px_)
            {
                state_ = ChaseState::ALIGNING;
                pid_linear_.reset();
            }
        }
        else // SEARCHING baru dapet target lagi -> mulai dari align
        {
            state_ = ChaseState::ALIGNING;
        }

        geometry_msgs::msg::Twist cmd;

        switch (state_)
        {
            case ChaseState::ALIGNING:
                // murni muter, gak maju sama sekali
                cmd.angular.z = pid_angular_.compute(-error_x, dt);
                cmd.linear.x = 0.0;
                break;

            case ChaseState::APPROACHING:
                // murni maju lurus, gak muter lagi
                cmd.angular.z = 0.0;
                cmd.linear.x = pid_linear_.compute(error_area, dt);
                break;

            case ChaseState::REACHED:
                cmd.angular.z = 0.0;
                cmd.linear.x = 0.0;
                pid_angular_.reset();
                pid_linear_.reset();
                break;

            default:
                break;
        }

        pub_cmd_->publish(cmd);

        RCLCPP_INFO(get_logger(), "err_x=%d  err_area=%d", error_x, error_area);
    }

    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_cmd_->publish(cmd);
    }

    // ================== MEMBER ==================
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_pos_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

    double x_, y_, yaw_;
    bool has_pos_;
    int robot_marker_id_;
    double aruco_yaw_offset_;
    bool aruco_yaw_invert_;
    cv::Mat homography_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_yellow_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_yellow_time_;

    ChaseState state_;
    bool yellow_detected_;
    int yellow_cx_;
    int yellow_area_;

    int image_width_;
    int target_area_;
    int area_tolerance_;
    int align_threshold_px_;
    int realign_threshold_px_;
    int detection_timeout_ms_;

    PIDController pid_angular_;
    PIDController pid_linear_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFusion>());
    rclcpp::shutdown();
    return 0;
}