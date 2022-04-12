#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "expand_custom_interfaces/msg/address_book.hpp"    // include header of newly created msg

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
    AddressBookPublisher() : Node("address_book_publisher") {
        addressBookPub_ = this->create_publisher<expand_custom_interfaces::msg::AddressBook>("address_book", 10);

        // a call back to publish messages periodically
        auto publishMsg = [this]() -> void {
            auto message = expand_custom_interfaces::msg::AddressBook();

            message.first_name = "John";
            message.last_name = "Doe";
            message.age = 30;
            message.gender = message.MALE;
            message.address = "unknown";

            std::cout << "Publishing Contact\nFirst:" << message.first_name <<
            "  Last:" << message.last_name << std::endl;

            this->addressBookPub_->publish(message);            
        };
        timer_ = this->create_wall_timer(1s, publishMsg);
    }

private:
    rclcpp::Publisher<expand_custom_interfaces::msg::AddressBook>::SharedPtr addressBookPub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookPublisher>());
    rclcpp::shutdown();

    return 0;
}