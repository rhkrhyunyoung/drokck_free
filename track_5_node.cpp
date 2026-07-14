#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class Track5FullIntegrated : public rclcpp::Node {
public:
    Track5FullIntegrated() : Node("Track_5_Full_Integrated") {
        string_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object", 10, std::bind(&Track5FullIntegrated::label_callback, this, std::placeholders::_1));
        xyz_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/yolo_object_xyz", 10, std::bind(&Track5FullIntegrated::xyz_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&Track5FullIntegrated::image_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&Track5FullIntegrated::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "=== mission5 start! ===");
    }

private:
    void label_callback(const std_msgs::msg::String::SharedPtr msg) {
        is_red_flag_ = (msg->data == "STOP" || msg->data == "red_flag");
    }
    void xyz_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        target_distance_ = msg->vector.x;
        last_yolo_time_ = this->now();
    }
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            int width = frame.cols;
            int height = frame.rows;

            cv::Mat roi = frame(cv::Rect(0, height * 2 / 3, width, height / 3));

            cv::Mat gray, binary;
            cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV); 

            cv::Moments m = cv::moments(binary, true);
            if (m.m00 > 0) {
                double cx = m.m10 / m.m00;
                line_error_ = (cx - (width / 2)) / (width / 2);
                last_line_time_ = this->now();
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void control_loop() {
        auto out_msg = geometry_msgs::msg::Twist();
        auto now = this->now();

        if ((now - last_line_time_).seconds() < 0.5) {
            double k_angular = 1.8;
            out_msg.angular.z = -line_error_ * k_angular;
        }

        double dt_yolo = (now - last_yolo_time_).seconds();

        if (is_red_flag_ && target_distance_ < 1.2) {
            out_msg.linear.x = 0.0;
        } 
        else if (dt_yolo < 0.5) {
            double error = target_distance_ - 2.0;
            double k_linear = 0.7;
            out_msg.linear.x = 0.4 + (error * k_linear);
        } 
        else if (dt_yolo < 3.0) {
            out_msg.linear.x = 0.3;
        } 
        else {
            out_msg.linear.x = 0.0;
        }
        if (out_msg.linear.x > 0.8) out_msg.linear.x = 0.8;
        if (out_msg.linear.x < 0.0) out_msg.linear.x = 0.0;

        cmd_pub_->publish(out_msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr xyz_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_red_flag_ = false;
    double target_distance_ = 0.0;
    double line_error_ = 0.0;
    rclcpp::Time last_yolo_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_line_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Track5FullIntegrated>());
    rclcpp::shutdown();
    return 0;
}
