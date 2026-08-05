#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class WinterNode : public rclcpp::Node {
public:
    WinterNode() : Node("Winter_Node") {
        sub_l.subscribe(this, "/camera_left/color/image_raw");
        sub_f.subscribe(this, "/camera_front/color/image_raw");
        sub_r.subscribe(this, "/camera_right/color/image_raw");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_l, sub_f, sub_r);
        sync_->registerCallback(std::bind(&WinterNode::combined_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg){ yaw_rate = msg->angular_velocity.z; });
        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg){ current_nav = *msg; });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
private:
    void combined_callback(const sensor_msgs::msg::Image::ConstSharedPtr& ml, const sensor_msgs::msg::Image::ConstSharedPtr& mf, const sensor_msgs::msg::Image::ConstSharedPtr& mr) {
        auto cv_l = cv_bridge::toCvCopy(ml, "bgr8")->image;
        auto cv_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
        auto cv_r = cv_bridge::toCvCopy(mr, "bgr8")->image;
        cv::resize(cv_l, cv_l, cv::Size(400,300)); cv::resize(cv_f, cv_f, cv::Size(400,300)); cv::resize(cv_r, cv_r, cv::Size(400,300));
        cv::Mat combined; cv::hconcat(std::vector<cv::Mat>{cv_l, cv_f, cv_r}, combined);

        auto out_msg = current_nav;
        // 슬립 감지: 명령한 회전량 대비 실제 IMU 회전량이 부족할 때
        if (std::abs(out_msg.linear.x) > 0.05 && std::abs(out_msg.angular.z - yaw_rate) > 0.3) {
            out_msg.linear.x *= 3.0; // 탈출을 위해 출력 강화
            cv::putText(combined, "SLIP DETECTED!", cv::Point(450, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
        } else { out_msg.linear.x *= 1.2; }
        
        cmd_pub_->publish(out_msg);
        cv::imshow("Winter_3Cam_Vision", combined); cv::waitKey(1);
    }
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_l, sub_f, sub_r;
    geometry_msgs::msg::Twist current_nav; double yaw_rate = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<WinterNode>()); rclcpp::shutdown(); return 0; }
