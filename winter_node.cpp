#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), sub_l, sub_f, sub_r);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.5));
        sync_->registerCallback(std::bind(&WinterNode::process_all, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            this->current_yaw_rate = msg->angular_velocity.z;
        });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->current_nav = *msg; });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);
    }

private:
    void process_all(const sensor_msgs::msg::Image::ConstSharedPtr& ml, const sensor_msgs::msg::Image::ConstSharedPtr& mf, const sensor_msgs::msg::Image::ConstSharedPtr& mr) {
        try {
            cv::Mat img_l = cv_bridge::toCvCopy(ml, "bgr8")->image;
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::Mat img_r = cv_bridge::toCvCopy(mr, "bgr8")->image;

            cv::Size target_size(640, 480);
            cv::resize(img_l, img_l, target_size); cv::resize(img_f, img_f, target_size); cv::resize(img_r, img_r, target_size);

            cv::Mat combined;
            cv::hconcat(std::vector<cv::Mat>{img_l, img_f, img_r}, combined);

            // 겨울 슬립 보정 로직
            auto out_msg = current_nav;
            if (std::abs(out_msg.linear.x) > 0.05 && std::abs(out_msg.angular.z - current_yaw_rate) > 0.3) {
                out_msg.linear.x *= 3.0; // 탈출 토크
                cv::putText(combined, "SLIP DETECTED!", cv::Point(500, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
            } else {
                out_msg.linear.x *= 1.1; // 기본 겨울 주행력 강화
            }
            cmd_pub_->publish(out_msg);

            cv::Mat ui_view; cv::resize(combined, ui_view, cv::Size(), 0.5, 0.5);
            auto msg = cv_bridge::CvImage(ml->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);
            cv::imshow("Winter_Integrated_Vision", ui_view); cv::waitKey(1);
        } catch (cv::Exception& e) { RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what()); }
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_l, sub_f, sub_r;
    geometry_msgs::msg::Twist current_nav;
    double current_yaw_rate = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<WinterNode>()); rclcpp::shutdown(); return 0; }
