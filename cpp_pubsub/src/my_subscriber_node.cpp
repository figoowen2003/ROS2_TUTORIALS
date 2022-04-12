// Include the C++ standard library headers
#include <memory> // Dynamic memory management
 
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
// #include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
// for custom msg test
#include "tutorial_interfaces/msg/num.hpp"
using std::placeholders::_1;
 
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // Constructor
    // The name of the node is minimal_subscriber
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      // Create the subscription.
      // The TopicCallback function executes whenever data is published
      // to the 'ROS' topic.
      // subscription_ = this->create_subscription<std_msgs::msg::String>(
      // "ROS", 10, std::bind(&MinimalSubscriber::TopicCallback, this, _1));

      // for custom msg test
      subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>("ROS",
      10, std::bind(&MinimalSubscriber::TopicCallback, this, _1));
    }
 
  private:
    // Receives the String message that is published over the topic
    // void TopicCallback(const std_msgs::msg::String::SharedPtr msg) const
    // {
    //   // Write the message that was received on the console window
    //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    // }

    // recieves custom msg published over topic
    void TopicCallback(const tutorial_interfaces::msg::Num & msg) const  // CHANGE
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");     // CHANGE
    }    
    // Declare the subscription attribute
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    // declare for custom msg test
    rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};
 
int main(int argc, char * argv[])
{
  // Launch ROS 2
  rclcpp::init(argc, argv);
   
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}