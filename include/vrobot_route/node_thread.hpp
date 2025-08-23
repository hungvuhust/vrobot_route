#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace vrobot_route {
/**
 * @class vrobot_route::NodeThread
 * @brief A background thread to process node/executor callbacks
 */
class NodeThread {
public:
  /**
   * @brief A background thread to process node callbacks constructor
   * @param node_base Interface to Node to spin in thread
   */
  explicit NodeThread(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
    : node_(node_base) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_   = std::make_unique<std::thread>([&]() {
      executor_->add_node(node_);
      executor_->spin();
      executor_->remove_node(node_);
    });
  }

  /**
   * @brief A background thread to process executor's callbacks constructor
   * @param executor Interface to executor to spin in thread
   */
  explicit NodeThread(
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
    : executor_(executor) {
    thread_ = std::make_unique<std::thread>([&]() { executor_->spin(); });
  }

  /**
   * @brief A background thread to process node callbacks constructor
   * @param node Node pointer to spin in thread
   */
  template <typename NodeT>
  explicit NodeThread(NodeT node)
    : NodeThread(node->get_node_base_interface()) {
  }

  /**
   * @brief A destructor
   */
  ~NodeThread() {
    executor_->cancel();
    thread_->join();
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread>                          thread_;
  rclcpp::Executor::SharedPtr                           executor_;
};

}  // namespace vrobot_route
