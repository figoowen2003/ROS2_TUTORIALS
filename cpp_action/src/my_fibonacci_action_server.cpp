#include <functional>
#include <memory>
#include <thread>

// #include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "tutorial_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace my_action_cpp
{
class FibonacciActionServer : public rclcpp::Node {
public:
  using Fibonacci = tutorial_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit FibonacciActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
    : Node("fibonacci_action_server", options) {
      using namespace std::placeholders;

      actionServer_ = rclcpp_action::create_server<Fibonacci>(
        this,   // node
        "fibonacci",  // action name 
        std::bind(&FibonacciActionServer::HandleGoal, this, _1, _2), 
        std::bind(&FibonacciActionServer::HandleCancel, this, _1), 
        std::bind(&FibonacciActionServer::HandleAccept, this, _1));
    }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr actionServer_;

  // 接收所有的goals
  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID &uuid, 
    std::shared_ptr<const Fibonacci::Goal> goal
  ) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleFibonacci> goalHandle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccept(const std::shared_ptr<GoalHandleFibonacci> goalHandle) {
    using namespace std::placeholders;
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goalHandle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goalHandle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // 用于提供sleep操作
    rclcpp::Rate loopRate(1);
    const auto goal = goalHandle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();    // 反馈信息
    auto &sequence = feedback->partial_sequence;    // 反馈信息里定义的序列，用于存放中间结果
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();    // 最终返回结果

    // 计算每一阶的fabonnci值，直到获取最终结果
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
        // 如果捕获到了一个cancel的申请，那么需要返回最后一次计算的结果
        if (goalHandle->is_canceling()) {
            result->sequence = sequence;
            goalHandle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // 持续更新Sequence
        sequence.push_back(sequence[i - 1] + sequence[i]);
        goalHandle->publish_feedback(feedback);    // 持续发布反馈给客户端，间隔时间为1s
        RCLCPP_INFO(this->get_logger(), "Publiser feedback");

        loopRate.sleep();
    }

    // 如果计算全部完成，发布最终结果
    if (rclcpp::ok()) {
        result->sequence = sequence;
        goalHandle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }            
  }

};  // class 
} // namespace

// 通过这个宏将action server注册为一个组件，以便运行时动态加载
RCLCPP_COMPONENTS_REGISTER_NODE(my_action_cpp::FibonacciActionServer)

