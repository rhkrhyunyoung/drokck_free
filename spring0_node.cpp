#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SpringNode : public rclcpp::Node {
public:
    SpringNode() : Node("Spring_Node") {
        // 1. 구독 설정 (리얼센스 IMU 토픽 경로 적용)
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&SpringNode::process_front, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/camera/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_imu_linear_x = msg->linear_acceleration.x;
            });

        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("ally") != std::string::npos) RCLCPP_INFO(this->get_logger(), ">> ALLY FOUND!");
                else if (msg->data.find("enemy") != std::string::npos) RCLCPP_ERROR(this->get_logger(), ">> ENEMY FOUND!");
            });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
       
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);

        RCLCPP_INFO(this->get_logger(), "=== [SPRING] TANK RECOVERY (3s DELAY) NODE STARTING ===");
    }

private:
    void process_front(const sensor_msgs::msg::Image::SharedPtr mf)
    {
        try {
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::Size target_size(640, 480);
            cv::resize(img_f, img_f, target_size);

            auto out_msg = current_nav;
            auto now = this->get_clock()->now();

            // --- 끼임 감지 및 탈출 로직 ---

            // 1. 끼임 감지 조건 확인 (네비 명령은 있는데 가속도가 0.01 이하)
            bool is_stuck_now = (std::abs(current_nav.linear.x) > 0.1 && std::abs(current_imu_linear_x) <= 0.01);

            if (is_stuck_now && !is_recovering) {
                if (!is_potential_stuck) {
                    is_potential_stuck = true;
                    stuck_start_time = now; // 끼임 의심 시작 시점 기록
                } else {
                    // 3초 이상 유지되었는지 확인
                    double stuck_duration = (now - stuck_start_time).seconds();
                    if (stuck_duration >= 3.0) {
                        is_recovering = true;
                        recovery_start_time = now; // 3초 탈출 모드 시작
                        is_potential_stuck = false;
                        RCLCPP_WARN(this->get_logger(), "Stuck maintained for 3s! Starting Recovery Spin...");
                    }
                }
            } else {
                // 가속도가 다시 붙으면 끼임 의심 상태 해제
                is_potential_stuck = false;
            }

            // 2. 탈출 동작 수행 (3초간)
            if (is_recovering) {
                double recovery_elapsed = (now - recovery_start_time).seconds();
               
                if (recovery_elapsed < 3.0) {
                    out_msg.linear.x = 0.0; // 전진 정지
                   
                    // 네비게이션이 가려던 방향으로 강하게 회전
                    if (current_nav.angular.z > 0.05) {
                        out_msg.angular.z = 1.2;
                    } else if (current_nav.angular.z < -0.05) {
                        out_msg.angular.z = -1.2;
                    } else {
                        out_msg.angular.z = 0.8; // 직진 중이었다면 임의 회전
                    }
                } else {
                    is_recovering = false;
                    RCLCPP_INFO(this->get_logger(), "Recovery Finished. Resuming normal drive.");
                }
            } else {
                // 일반 주행 (60% 서행 유지)
                out_msg.linear.x *= 0.6;
            }

            cmd_pub_->publish(out_msg);

            // --- 시각화 ---
            cv::Mat ui_view;
            cv::resize(img_f, ui_view, cv::Size(), 0.5, 0.5);
            if (is_recovering) {
                cv::putText(ui_view, "STUCK! RECOVERY", cv::Point(10, 30), 2, 0.7, cv::Scalar(0, 0, 255), 2);
            } else if (is_potential_stuck) {
                cv::putText(ui_view, "STUCK CHECKING...", cv::Point(10, 30), 2, 0.7, cv::Scalar(0, 255, 255), 2);
            }

            auto msg = cv_bridge::CvImage(mf->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);
            cv::imshow("Front_Cam_Vision", ui_view);
            cv::waitKey(1);

        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what());
        }
    }

    // 변수 선언
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;

    geometry_msgs::msg::Twist current_nav;
    double current_imu_linear_x = 0.0;

    // 상태 관리 변수
    bool is_recovering = false;
    bool is_potential_stuck = false;
    rclcpp::Time stuck_start_time;
    rclcpp::Time recovery_start_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpringNode>());
    rclcpp::shutdown();
    return 0;
}
