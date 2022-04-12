#include <chrono>

#include <example_interfaces/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node {
public:
    DataGenerator() : Node("data_generator") {
        data_.data = 0;

        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open("timed_synthetic_bag");
        writer_->create_topic(  // 提前将topic以及其类型注册到bag中，在传递没有topic的序列化消息时必须有此操作
            {
                "synthetic",
                "example_interfaces/msg/Int32",
                rmw_get_serialization_format(),
                ""
            }
        );

        timer_ = this->create_wall_timer(1s, std::bind(&DataGenerator::TimerCallback, this));
    }

private:
    void TimerCallback() {
        // 此时数据还没有序列化
        writer_->write(data_, "synthetic", now());

        ++data_.data;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    example_interfaces::msg::Int32 data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}