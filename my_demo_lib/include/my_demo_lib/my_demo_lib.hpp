#ifndef _MY_DEMO_LIB_HPP_
#define _MY_DEMO_LIB_HPP_

#include <rclcpp/rclcpp.hpp>

using namespace std;

namespace my_demo_lib {
class MyLib : public rclcpp::Node {
public:
    MyLib() : Node("my_demo_lib") {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2");
    }
private:

};  // Mylib
}   // my_demo_lib

#endif