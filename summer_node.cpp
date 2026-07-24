#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SummerNode : public rclcpp::Node {
public:
    SummerNode() : Node("Summer_Node") {
        sub_l.subscribe(this, "/camera_left/color/image_raw");
        sub_f.subscribe(this, "/camera_front/color/image_raw");
        sub_r.subscribe(this, "/camera_right/color/image_raw");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_l, sub_f, sub_r);
        sync_->registerCallback(std::bind(&SummerNode::combined_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>("/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg){
            if (msg->data.find("red_light") != std::string::npos) is_red = true;
            else if (msg->data.find("green_light") != std::string::npos) is_red = false;
            if (msg->data.find("supply_box") != std::string::npos && !is_wait) { is_wait = true; start_t = this->now(); }
        });
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

        auto out_msg = geometry_msgs::msg::Twist();
        if (is_red) { out_msg.linear.x = 0.0; } // 빨간불 정지
        else if (is_wait && (this->now() - start_t).seconds() < 20.0) { out_msg.linear.x = 0.0; } // 상자 대기
        else { is_wait = false; out_msg = current_nav; }
        
        cmd_pub_->publish(out_msg);
        cv::imshow("Summer_3Cam_Vision", combined); cv::waitKey(1);
    }
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_l, sub_f, sub_r;
    geometry_msgs::msg::Twist current_nav;
    bool is_red = false, is_wait = false; rclcpp::Time start_t;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<SummerNode>()); rclcpp::shutdown(); return 0; }
