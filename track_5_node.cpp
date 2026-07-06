#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class Track5PureFollower : public rclcpp::Node {
public:
    Track5PureFollower() : Node("Track_5_Pure_Follower") {
        // 1. YOLO 라벨 구독 (STOP, GO, robot_dog 등 상태 확인용)
        string_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object", 10, std::bind(&Track5PureFollower::label_callback, this, std::placeholders::_1));

        // 2. YOLO 3D 좌표 구독 (실제 거리 및 방향 확인용)
        xyz_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/yolo_object_xyz", 10, std::bind(&Track5PureFollower::xyz_callback, this, std::placeholders::_1));

        // 3. 차체 제어 명령 발행
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 안전을 위한 타이머 (데이터가 안 오면 0.5초 뒤 자동 정지)
        watchdog_timer_ = this->create_wall_timer(
            500ms, std::bind(&Track5PureFollower::safety_check, this));

        RCLCPP_INFO(this->get_logger(), "=== mission5 start ===");
    }

private:
    // 라벨 콜백 (빨간 가림막 감지 시 최우선 정지)
    void label_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "STOP" || msg->data == "red_flag") {
            is_red_flag_ = true;
        } else {
            is_red_flag_ = false;
        }
    }

    // 좌표 콜백 (여기서 실제 주행 계산)
    void xyz_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        last_update_time_ = this->now();
        target_detected_ = true;

        auto out_msg = geometry_msgs::msg::Twist();

        // 1. 빨간 가림막이 있으면 무조건 정지
        if (is_red_flag_) {
            stop_robot(out_msg);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "=== stop ===");
        } 
        // 2. 추종 로봇(robot_dog 등)이 감지되었을 때
        else {
            // q.py에서 정의한 대로: vector.x 가 정면 거리(Z), vector.y 가 좌우 편차(X)
            double dist = msg->vector.x;
            double offset_x = msg->vector.y;

            // --- 조향 제어 (Angular Z) ---
            // 타겟이 중앙에서 멀어질수록 강하게 회전
            double k_angular = 2.0; 
            out_msg.angular.z = offset_x * k_angular; 

            // --- 속도 제어 (Linear X: 2m 간격 유지) ---
            double target_gap = 2.0; 
            double error = dist - target_gap;
            double k_linear = 0.8; 

            out_msg.linear.x = error * k_linear;

            // 속도 제한 (로봇개 미션 속도 1.0m/s 이내로 제한)
            if (out_msg.linear.x > 1.0) out_msg.linear.x = 1.0;
            if (out_msg.linear.x < 0.0) out_msg.linear.x = 0.0; // 후진은 하지 않음

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                                "Follow -> Dist: %.2fm, Speed: %.2f", dist, out_msg.linear.x);
        }
        cmd_pub_->publish(out_msg);
    }

    // 세이프티 체크 (가림 구간 등으로 로봇이 안 보이면 정지)
    void safety_check() {
        auto now = this->now();
        if ((now - last_update_time_).seconds() > 0.5) {
            if (target_detected_) {
                auto out_msg = geometry_msgs::msg::Twist();
                stop_robot(out_msg);
                cmd_pub_->publish(out_msg);
                target_detected_ = false;
                RCLCPP_WARN(this->get_logger(), "=== waiting... ===");
            }
        }
    }

    void stop_robot(geometry_msgs::msg::Twist &msg) {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr xyz_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    bool is_red_flag_ = false;
    bool target_detected_ = false;
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Track5PureFollower>());
    rclcpp::shutdown();
    return 0;
}
