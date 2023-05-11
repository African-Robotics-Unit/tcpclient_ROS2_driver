#include "rclcpp/rclcpp.hpp"
#include "client_interfaces/srv/send_command_string_to_server.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <cstring>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

//   if (argc != 2) {
//       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: send_command_string_client str");
//       return 1;
//   }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("send_request_client");
  rclcpp::Client<client_interfaces::srv::SendCommandStringToServer>::SharedPtr client =
    node->create_client<client_interfaces::srv::SendCommandStringToServer>("send_request");

  auto request = std::make_shared<client_interfaces::srv::SendCommandStringToServer::Request>();
  //char commandString[] = "dfeDataOutputMode 1\n";
  std::string commandString = "dfeDataOutputMode 1\n";
  request->command = commandString;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sending command: " << request->command);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Response: "<< result.get()->reply);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service send_request");
  }

  rclcpp::shutdown();
  return 0;
}