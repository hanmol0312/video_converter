#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>
#include <atomic>

using namespace std;

class ImageConversionServer : public rclcpp::Node {
public:
    ImageConversionServer() : Node("image_conversion_server"), mode_(2) {


        this->declare_parameter<std::string>("input_image_topic", "web_cam/image_raw");
        this->declare_parameter<std::string>("output_image_topic", "web_cam/converted_image");

        string input_topic;
        this->get_parameter("input_image_topic", input_topic);
        string output_topic;
        this->get_parameter("output_image_topic", output_topic);


        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(input_topic, 10, std::bind(&ImageConversionServer::imageCallback, this, std::placeholders::_1));

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_image_mode", 
            std::bind(&ImageConversionServer::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::Mat processed_image;
            if (mode_ == 1) {

                cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);
                cv::cvtColor(processed_image, processed_image, cv::COLOR_GRAY2BGR);
            } else {

                processed_image = image;
            }



            image_publisher_->publish(*cv_bridge::CvImage(msg->header, "bgr8", processed_image).toImageMsg());

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
        }
    }

    void setModeCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                         std_srvs::srv::SetBool::Response::SharedPtr response) {
        mode_ = request->data ? 1 : 2;
        response->success = true;
        response->message = mode_ == 1 ? "Mode set to Grayscale" : "Mode set to Color";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
    int mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
