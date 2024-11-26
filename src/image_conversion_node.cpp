#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <iostream>

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode() : Node("image_conversion_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                               .reliable()
                               .transient_local();
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_ = this->create_client<std_srvs::srv::SetBool>("set_image_mode",rmw_qos_profile_services_default, client_callback_group_);
        RCLCPP_INFO(this->get_logger(), "User Interface Node started. Waiting for service...");
        
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        runUI();
    }

private:
    void runUI() {
        while (rclcpp::ok()) {
            std::cout << "\nSelect Image Mode:\n"
                      << "1. Grayscale\n"
                      << "2. Color\n"
                      << "0. Exit\n"
                      << "Enter your choice: ";
            
            int choice;
            std::cin >> choice;

            if (choice == 0) {
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                rclcpp::shutdown();
                break;
            }

            bool mode = (choice == 1); 

            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = mode;

            auto future = client_->async_send_request(
                request, 
                std::bind(&ImageConversionNode::handleServiceResponse, this, std::placeholders::_1)
            );

        }

    }

    void handleServiceResponse(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future) {
        try {
            auto response = response_future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to set mode.");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }


    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
