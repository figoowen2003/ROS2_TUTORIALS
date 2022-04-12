#include <memory>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/node_factory.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);
    vector<rclcpp_components::NodeInstanceWrapper> nodeWrappers;

    // using rclcpp_components::ComponentManager::ComponentResource = std::pair<std::string, std::string>
    // 表示一个组件资源，第一个参数是class name（for class loader）, 第二个参数是library path（绝对路径）
    rclcpp_components::ComponentManager::ComponentResource componentTalker("composition::Talker", "/opt/ros/galactic/lib/libtalker_component.so");
    // std::shared_ptr< rclcpp_components::NodeFactory > 	create_component_factory (const ComponentResource &resource)
    auto talkerFactory = manager->create_component_factory(componentTalker);
    auto talkerwrapper = talkerFactory->create_node_instance(options);
    auto talkernode = talkerwrapper.get_node_base_interface();
    nodeWrappers.push_back(talkerwrapper);
    exec->add_node(talkernode);

    rclcpp_components::ComponentManager::ComponentResource componentListener("composition::Listener", "/opt/ros/galactic/lib/liblistener_component.so");
    auto listenerFactory = manager->create_component_factory(componentListener);
    auto listenerwrapper = listenerFactory->create_node_instance(options);
    auto listenernode = listenerwrapper.get_node_base_interface();
    nodeWrappers.push_back(listenerwrapper);
    exec->add_node(listenernode);    

    // Return a list of valid loadable components in a given package.
    auto resources = manager->get_component_resources("composition");
    for (const auto &cmp : resources) {
        std::cout << "[component in manager]:  " << cmp.first << std::endl;
    }

    exec->spin();

    for (auto wrapper : nodeWrappers) {
        exec->remove_node(wrapper.get_node_base_interface());
    }
    nodeWrappers.clear();

    rclcpp::shutdown();

    return 0;
}