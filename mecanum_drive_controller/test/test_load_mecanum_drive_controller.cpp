#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadMecanumDriveController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf),
    executor, "test_controller_manager");

  const std::string test_file_path =
    std::string(TEST_FILES_DIRECTORY) + "/mecanum_drive_controller_params.yaml";

  cm.set_parameter({"test_mecanum_drive_controller.params_file", test_file_path});
  cm.set_parameter(
    {"test_mecanum_drive_controller.type", "mecanum_drive_controller/MecanumDriveController"});

  ASSERT_NE(cm.load_controller("test_mecanum_drive_controller"), nullptr);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
