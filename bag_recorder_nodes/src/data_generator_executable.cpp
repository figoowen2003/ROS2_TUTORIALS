#include <chrono>

#include <rclcpp/rclcpp.hpp>    // 用于使用rclcpp::Clock, rclcpp::Duration 和 rclcpp::Time
#include <rosbag2_cpp/writer.hpp>
// #include <rosbag2_cpp/writers/sequential_writer.hpp>     // useless
// #include <rosbag2_storage/serialized_bag_message.hpp>    // useless
#include <example_interfaces/msg/int32.hpp>

using namespace std::chrono_literals;

int main(int, char**) {
    example_interfaces::msg::Int32 data;
    data.data = 0;

    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open("big_synthetic_bag");
    writer->create_topic(
        {
            "synthectic",
            "example_interfaces/msg/Int32",
            rmw_get_serialization_format(),
            ""
        }
    );

    rclcpp::Clock clock;
    rclcpp::Time timeStamp = clock.now();
    for (int32_t i = 0; i < 100; ++i) {
        writer->write(data, "synthetic", timeStamp);
        ++data.data;
        timeStamp += rclcpp::Duration(1s);  // 时间戳可以是所需要的任意数值
    }

    return 0;
}