#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class WinterNode : public rclcpp::Node {
public:
    WinterNode() : Node("Winter_Node") {
        // 1. 전방 카메라만 구독 (가운데만 사용)
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, 
            std::bind(&WinterNode::process_front, this, std::placeholders::_1));

        // 2. IMU 데이터 구독 (Yaw rate 및 Linear Acceleration X 추출)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_yaw_rate = msg->angular_velocity.z;
                this->current_accel_x = msg->linear_acceleration.x; // 가속도 X 저장
            });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { 
                this->current_nav = *msg; 
            });

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);
    }

private:
    void process_front(const sensor_msgs::msg::Image::SharedPtr mf) {
        try {
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::Size target_size(640, 480);
            cv::resize(img_f, img_f, target_size);

            // 주행 제어 로직
            auto out_msg = current_nav;

            // [기존 겨울 슬립 보정 로직]
            if (std::abs(out_msg.linear.x) > 0.05 && std::abs(out_msg.angular.z - current_yaw_rate) > 0.3) {
                out_msg.linear.x *= 3.0; // 탈출 토크
                cv::putText(img_f, "SLIP DETECTED!", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
            } else {
                out_msg.linear.x *= 1.1; // 기본 겨울 주행력 강화
            }

            // [추가 요청 로직] IMU 가속도 x가 0.1보다 작으면 토크 2배
            if (std::abs(current_accel_x) < 0.1) {
                out_msg.linear.x *= 2.0;
                cv::putText(img_f, "ACCEL BOOST (x2)", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0), 2);
            }

            cmd_pub_->publish(out_msg);

            // UI 및 시각화
            cv::Mat ui_view; 
            cv::resize(img_f, ui_view, cv::Size(), 0.5, 0.5);
            auto msg = cv_bridge::CvImage(mf->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);
            
            cv::imshow("Winter_Front_Vision", ui_view); 
            cv::waitKey(1);

        } catch (cv::Exception& e) { 
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what()); 
        }
    }

    // 전방 카메라 전용 구독자
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;

    geometry_msgs::msg::Twist current_nav;
    double current_yaw_rate = 0.0;
    double current_accel_x = 0.0; // 가속도 값 저장용
};

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<WinterNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}
