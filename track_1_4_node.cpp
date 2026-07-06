#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <algorithm>
#include <string>

using namespace std::chrono_literals;

enum class MyMissionState {
    TRACK_DRIVING,
    TRAFFIC_STOP,
    SUPPLY_BOX
};

class MissionManager : public rclcpp::Node {
public:
    MissionManager() : Node("Mission_Manager_Node") {
        // 라인트레이싱 노드가 보내는 원본 신호 (Python 코드에서 /cmd_vel_nav로 쏴줘야 함)
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&MissionManager::nav_callback, this, std::placeholders::_1));
        
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object", 10, std::bind(&MissionManager::yolo_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_yaw_rate_ = msg->angular_velocity.z;
            });

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        current_state_ = MyMissionState::TRACK_DRIVING;
        g_traffic_signal_stop = false;
        is_supply_box_waiting = false;
        is_slipping = false;
        torque_multiplier = 1.0;
        current_yaw_rate_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "=== mission start ===");
    }

private:
    void yolo_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string label = msg->data;
        
        if (label == "STOP") {
            if (!g_traffic_signal_stop) {
                RCLCPP_ERROR(this->get_logger(), "=== STOP ===");
                g_traffic_signal_stop = true;
            }
        } else if (label == "GO") {
            if (g_traffic_signal_stop) {
                RCLCPP_INFO(this->get_logger(), "=== GO ===");
                g_traffic_signal_stop = false;
            }
        } else if (label == "supply_box") {
            if (!is_supply_box_waiting) {
                is_supply_box_waiting = true;
                supply_box_start_time = this->now();
                RCLCPP_WARN(this->get_logger(), "=== Supply Box ===");
            }
        }
    }

    void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto out_msg = geometry_msgs::msg::Twist();
        double raw_linear = msg->linear.x;
        double raw_angular = msg->angular.z;

        // 1. 신호등 체크 (최우선 순위 정지)
        if (g_traffic_signal_stop) {
            out_msg.linear.x = 0.0;
            out_msg.angular.z = 0.0;
        }
        // 2. 보급 상자 대기 체크
        else if (is_supply_box_waiting) {
            double elapsed = (this->now() - supply_box_start_time).seconds();
            if (elapsed < 20.0) {
                out_msg.linear.x = 0.0;
                out_msg.angular.z = 0.0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "=== Waiting... (%.1f/20s) ===", elapsed);
            } else {
                RCLCPP_INFO(this->get_logger(), "=== Done ===");
                is_supply_box_waiting = false;
            }
        }
        // 3. 정상 주행 및 슬립 보정
        else {
            // 슬립 보정 로직
            if (std::abs(raw_linear) > 0.05 && std::abs(raw_angular - current_yaw_rate_) > 0.3) {
                if (!is_slipping) {
                    is_slipping = true;
                    slip_start_time = this->now();
                } else {
                    double dur = (this->now() - slip_start_time).seconds();
                    if (dur > 3.0) {
                        torque_multiplier = std::min(4.0, 1.0 + (dur - 3.0) * 0.5);
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "=== Slip Correction : %.1fx ===", torque_multiplier);
                    }
                }
            } else {
                is_slipping = false;
                torque_multiplier = 1.0;
            }

            out_msg.linear.x = raw_linear * torque_multiplier;
            out_msg.angular.z = raw_angular;
        }

        // 최종 제어 신호를 차체 드라이버(/cmd_vel)로 발행
        cmd_pub_->publish(out_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    MyMissionState current_state_;
    bool g_traffic_signal_stop;
    bool is_supply_box_waiting;
    bool is_slipping;
    double current_yaw_rate_;
    double torque_multiplier;
    rclcpp::Time supply_box_start_time;
    rclcpp::Time slip_start_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
