#include "rclcpp/rclcpp.hpp" // ROS 2 C++ Client Library
#include "example_interfaces/srv/add_two_ints.hpp" // Package dependency
#include "tutorial_interfaces/srv/add_three_ints.hpp" // for custom srv test
 
#include <memory>
 
// void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
//           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
// {
//   // Adds two integers from the request and gives the sum to the response.
//   response->sum = request->a + request->b;
   
//   // Notifies the console of its status using logs.
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
//                 request->a, request->b);
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
// }

// for custom srv test
void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
  std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response) {  // CHANGE

  response->sum = request->a + request->b + request->c;                                      // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  // CHANGE
                request->a, request->b, request->c);                                         // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
 
int main(int argc, char **argv) {
  // Initialize the ROS 2 C++ Client Library
  rclcpp::init(argc, argv);
 
  // Create a node named add_two_ints_server
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");
 
  // Create a service named add_two_ints and advertise it over the network (i.e. &add method)
  // rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
  //   node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  // for custom srv test
  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service = 
    node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints", add);
 
  // Display a log message when the service is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");
 
  // Make the service available.
  rclcpp::spin(node);
   
  // Call shutdown procedure when we are done
  rclcpp::shutdown();
}