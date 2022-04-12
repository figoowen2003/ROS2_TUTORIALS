#include <memory>
#include <regex>

#include <rclcpp/rclcpp.hpp>

class MonitorNodeWithParams : public rclcpp::Node {
public:
    MonitorNodeWithParams() : Node("monitor_node_with_params") {
        this->declare_parameter("my_age", 18);
        paramEventMonitor_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // 设计一个回调函数
        auto cb = [this](const rclcpp::Parameter &p) {
            RCLCPP_INFO(
            this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());           
        };

        // 此处没有提供第三个参数（node name），将默认监控当前节点自身的参数变化
        paramCallback_ = paramEventMonitor_->add_parameter_callback("my_age", cb);

        // Now, add a callback to monitor any changes to the remote node's parameter. In this
        // case, we supply the remote node name.
        auto cb2 = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
            this->get_logger(), "cb2: Received an update to parameter \"%s\" of type: %s: \"%s\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_string().c_str());
        };
        auto remote_node_name = std::string("params_node");
        auto remote_param_name = std::string("my_param");
        otherNodeParamCallback_ = paramEventMonitor_->add_parameter_callback(remote_param_name, cb2, remote_node_name);

        // 监控所有参数变化
        auto cb3 =
            [remote_node_name, remote_param_name, this](const rcl_interfaces::msg::ParameterEvent & event) {
            // 使用正则表达式来对过滤出所希望监控的参数
            std::regex re("(/a_namespace/.*)|(/params_node)");
            if (regex_match(event.node, re)) {
                // 如果知道节点名和参数名，可以使用use 'get_parameter_from_event'
                rclcpp::Parameter p;
                if (rclcpp::ParameterEventHandler::get_parameter_from_event(
                    event, p,
                    remote_param_name, remote_node_name))
                {
                  RCLCPP_INFO(
                    this->get_logger(), "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
                    p.get_name().c_str(),
                    p.get_type_name().c_str(),
                    p.as_string().c_str());
                }

                // 使用'get_parameter*s*_from_event' 监控所有可以触发event的参数变化
                auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
                for (auto & p : params) {
                RCLCPP_INFO(
                    this->get_logger(), "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
                    p.get_name().c_str(),
                    p.get_type_name().c_str(),
                    p.value_to_string().c_str());
                }
            }
        };
        AllParamsCallback_ = paramEventMonitor_->add_parameter_event_callback(cb3);

    }
private:
    // ParameterEventHandler类监视节点参数的更改（自己的或者系统中其他节点的)，
    // 可以设置多个参数回调，当指定参数发生变化时它将被调用
    std::shared_ptr<rclcpp::ParameterEventHandler> paramEventMonitor_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> paramCallback_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> otherNodeParamCallback_;  // Add this
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> AllParamsCallback_;  // 必须将回调句柄设置为成员变量，保证它的生命周期与节点一致
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNodeWithParams>());
  rclcpp::shutdown();

  return 0;
}