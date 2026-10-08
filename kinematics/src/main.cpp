#include <rclcpp/rclcpp.hpp>

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode() : Node("kinematics_node")
  {
    RCLCPP_INFO(this->get_logger(), "kinematics_node berjalan (belum ada program)");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
