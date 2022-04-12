#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;   // 表示使用chrono字面量，也就是函数参数中1s这种写法

class ParamsNode: public rclcpp::Node
{
public:
    ParamsNode() : Node("params_node") {
        this->declare_parameter<std::string>("my_param", "ROS2");
        timer_ = this->create_wall_timer(1s, std::bind(&ParamsNode::respond, this));
    }

    void respond() {
        this->get_parameter("my_param", myParam_);
        RCLCPP_INFO(this->get_logger(), "Hello %s", myParam_.c_str());
    }

private:
    std::string myParam_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamsNode>());
    rclcpp::shutdown();

    return 0;
}