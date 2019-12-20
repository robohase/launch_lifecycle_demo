
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

class LifecycleDemo : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleDemo(const std::string &node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
                                        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "-- Configuring -->");
    
    std::this_thread::sleep_for(1s);
    
    return CallbackReturn::SUCCESS;
    // return CallbackReturn::FAILURE;
    // return CallbackReturn::ERROR;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "-- Activating -->");
    
    std::this_thread::sleep_for(1s);
    
    return CallbackReturn::SUCCESS;
    // return CallbackReturn::FAILURE;
    // return CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "-- Deactivating -->");
    
    std::this_thread::sleep_for(1s);
    
    return CallbackReturn::SUCCESS;
    // return CallbackReturn::ERROR;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {

    RCUTILS_LOG_INFO_NAMED(get_name(), "-- CleaningUp -->");
    
    std::this_thread::sleep_for(1s);
    
    return CallbackReturn::SUCCESS;
    // return CallbackReturn::ERROR;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
  {

    RCUTILS_LOG_INFO_NAMED(get_name(), "-- ShuttingDown from %s -->", state.label().c_str());
    
    std::this_thread::sleep_for(1s);
    
    return CallbackReturn::SUCCESS;
    // return CallbackReturn::ERROR;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &state)
  {

    RCUTILS_LOG_INFO_NAMED(get_name(), "-- ErrorProcessing from %s -->", state.label().c_str());
    
    std::this_thread::sleep_for(1s);

    return CallbackReturn::SUCCESS;
    // return CallbackReturn::FAILURE;
  }

};

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<LifecycleDemo> lc_demo = std::make_shared<LifecycleDemo>("demo_node");

  executor.add_node(lc_demo->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
