#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>   // 提供处理bag所需的函数与数据结构

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node {
public:
    SimpleBagRecorder() : Node("simple_bag_recorder") {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open("my_bag");    // 只提供bag的URL，其他选项使用默认值

        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, 
            std::bind(&SimpleBagRecorder::topicCallback, this, _1));
    }

private:
    // 区别与普通的回调函数，它接收一个rclcpp::SerializedMessage类型的消息
    void topicCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
        rclcpp::Time timeStamp = this->now();   // 必须确定用于存储消息的时间戳，通常选择数据产生的时间或者它被接收的时间，此处是后者。
        // 在demo中并没有注册任何topic，必须指定消息的完整的topic信息，所以需要提供topic name和topic type
        writer_->write(*msg, "chatter", "std_msgs/msg/String", timeStamp);
    }

    // 订阅者的类型与回调参数的类型保持一致，而非topic类型
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr sub_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}