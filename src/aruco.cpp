#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class RobotLocalizer : public rclcpp::Node
{
public:
    RobotLocalizer() : Node("robot_localizer")
    {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        params_ = cv::aruco::DetectorParameters::create();

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&RobotLocalizer::imageCb, this, std::placeholders::_1));
        pub_pos_ = create_publisher<geometry_msgs::msg::Point>("/robot_position", 10);
        pub_debug_ = create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);
        RCLCPP_INFO(get_logger(), "Robot Localizer (ArUco) started");
    }

private:
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
          }
          cv::Mat frame = cv_ptr->image;

          std::vector<int> ids;
          std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, params_, rejected);

        if (!ids.empty()) {
            int idx = 0;
            for (size_t i = 0; i < ids.size(); ++i) {
                if (ids[i] == target_id_) { idx = static_cast<int>(i); break; }
            }

            const auto &c = corners[idx];
            float cx = (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f;
            float cy = (c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f;

            double yaw = std::atan2(c[1].y - c[0].y, c[1].x - c[0].x);

            int h = frame.rows;
            int w = frame.cols;
            double x_m = (static_cast<double>(cx) / w) * field_size_ - field_offset_;
            double y_m = field_offset_ - (static_cast<double>(cy) / h) * field_size_;

            geometry_msgs::msg::Point pos;
            pos.x = x_m;
            pos.y = y_m;
            pos.z = 0.0;
            pub_pos_->publish(pos);

            // RCLCPP_INFO(get_logger(), "Marker id=%d pixel(%.0f,%.0f) -> world(%.2f, %.2f) yaw=%.2f",
            //             ids[idx], cx, cy, x_m, y_m, yaw);

            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            cv::circle(frame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame,
                "(" + std::to_string(x_m).substr(0,5) + ", " + std::to_string(y_m).substr(0,5) + ")",
                cv::Point(static_cast<int>(cx) + 10, static_cast<int>(cy)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        pub_debug_->publish(*debug_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_pos_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;

    const double field_size_ = 6.0;
    const double field_offset_ = 3.0;
    const int target_id_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotLocalizer>());
    rclcpp::shutdown();
    return 0;
}