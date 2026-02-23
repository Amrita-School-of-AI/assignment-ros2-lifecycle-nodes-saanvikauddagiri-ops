#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleSensor()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_sensor")
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Sensor Node created");
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 generator_{std::random_device{}()};
  std::uniform_real_distribution<double> distribution_{0.0, 100.0};

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/sensor_data", 10);

    RCLCPP_INFO(get_logger(), "Sensor configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();

    timer_ = this->create_wall_timer(
      500ms,
      [this]()
      {
        auto msg = std_msgs::msg::Float64();
        msg.data = distribution_(generator_);

        RCLCPP_INFO(this->get_logger(),
                    "Publishing sensor data: %.2f",
                    msg.data);

        publisher_->publish(msg);
      });

    RCLCPP_INFO(get_logger(), "Sensor activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_deactivate();
    timer_.reset();

    RCLCPP_INFO(get_logger(), "Sensor deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    publisher_.reset();
    RCLCPP_INFO(get_logger(), "Sensor cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Sensor shutting down");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleSensor>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
