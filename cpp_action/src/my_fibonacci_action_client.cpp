#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "tutorial_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace my_action_cpp {
class FibonacciActionClient : public rclcpp::Node {
public:
    using Fibonacci = tutorial_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : Node("fibonacci_action_client", options) {
            this->actionClient_ = rclcpp_action::create_client<Fibonacci>(this, 
                "fibonacci");

            this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                std::bind(&FibonacciActionClient::SendGoal, this));
    }

    void SendGoal() {
        using namespace std::placeholders;
        // 取消定时器，目的是定时器只被调用一次
        this->timer_->cancel();

        if (!this->actionClient_->wait_for_action_server()) {   // 等待服务器端可用
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            // return;
        }

        auto goalMsg = Fibonacci::Goal();
        goalMsg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto sendGoalOptions = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        sendGoalOptions.goal_response_callback = std::bind(&FibonacciActionClient::GoalResponseCallback, this, _1);
        sendGoalOptions.feedback_callback = std::bind(&FibonacciActionClient::FeedbackCallback, this, _1, _2);
        sendGoalOptions.result_callback = std::bind(&FibonacciActionClient::ResultCallback, this, _1);
        this->actionClient_->async_send_goal(goalMsg, sendGoalOptions);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr actionClient_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 当服务器收到并接受客户端发送的目标时，它将向客户端发送一个响应，本函数用于处理该响应
    void GoalResponseCallback(std::shared_future<GoalHandleFibonacci::SharedPtr> future) {
        auto goalHandle = future.get();
        if(!goalHandle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // 第一个参数无用（实参为何没有名称），第二个参数为客户端接收到的反馈信息
    void FeedbackCallback(GoalHandleFibonacci::SharedPtr, 
        const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void ResultCallback(const GoalHandleFibonacci::WrappedResult &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceld");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
    }
};  // FibonacciActionClient
}   // my_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(my_action_cpp::FibonacciActionClient)