#include <rclcpp/rclcpp.hpp>
#include "my_demo_lib/my_demo_lib.hpp"

using namespace std;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_demo_lib::MyLib>());
  rclcpp::shutdown();

  return 0;    
}