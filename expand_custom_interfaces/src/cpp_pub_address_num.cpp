#include <chrono>
#include <memory>
#include <typeinfo>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include "expand_custom_interfaces/msg/address_num.hpp"    // include header of newly created msg

using namespace std::chrono_literals;

class AddressNumPublisher : public rclcpp::Node
{
public:
    AddressNumPublisher() : Node("address_num_publisher") {
        addressBookPub_ = this->create_publisher<expand_custom_interfaces::msg::AddressNum>("address_num", 10);

        // a call back to publish messages periodically
        auto publishMsg = [this]() -> void {
        auto msg = std::make_shared<expand_custom_interfaces::msg::AddressNum>();
        {
            tutorial_interfaces::msg::Num nm;
            nm.num = 1;
            msg->address_nums.push_back(nm);
        }
        {
            tutorial_interfaces::msg::Num nm;
            nm.num = 2;
            msg->address_nums.push_back(nm);
        }

        std::cout << "Publishing address num:" << std::endl;
        for (auto contact : msg->address_nums) {
            // std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
            // std::endl;
            std::cout << "Address num: " << contact.num << std::endl;
        }

        addressBookPub_->publish(*msg);
        };
        timer_ = this->create_wall_timer(1s, publishMsg);
    }

private:
    rclcpp::Publisher<expand_custom_interfaces::msg::AddressNum>::SharedPtr addressBookPub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    std::array<uint8_t, 3> uuid{1, 2, 3};
    // (void)uuid;
    std::cout << "uuid type: " << typeid(uuid).name() << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressNumPublisher>());
    rclcpp::shutdown();

    return 0;
}