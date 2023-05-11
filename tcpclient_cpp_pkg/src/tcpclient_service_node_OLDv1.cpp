// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "client_interfaces/srv/send_command_string_to_server.hpp"

// Includes
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <memory>

// Definitions
#define PORT 8080
#define NANO_IP_ADDR "137.158.125.136"



void sendRequest(const std::shared_ptr<client_interfaces::srv::SendCommandStringToServer::Request> request,
          std::shared_ptr<client_interfaces::srv::SendCommandStringToServer::Response> response){
    const char* command = request->command.c_str();
    int sock = 0, client_fd;
    char buffer[1024] = { 0 };
    bool errorFlag = false;

    // define server address struct
    struct sockaddr_in serv_addr;

    // Create socket
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("nano_tcp_service"), "Could not create socket");
        errorFlag = true;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary
    const char* address = NANO_IP_ADDR;
    if (inet_pton(AF_INET, address, &serv_addr.sin_addr)<= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("nano_tcp_service"), "Invalid address or Address not supported");
        errorFlag = true;
    }

    if ((client_fd= connect(sock, (struct sockaddr*)&serv_addr,sizeof(serv_addr)))< 0) {
        RCLCPP_ERROR(rclcpp::get_logger("nano_tcp_service"), "Connection Failed");
        errorFlag = true;
    }

    std::string reply;
    if (errorFlag == true){
        RCLCPP_ERROR(rclcpp::get_logger("nano_tcp_service"), "Error trying to connect, cannot send command");
    } else {
        send(sock, command, strlen(command), 0);
        read(sock, buffer, 1024);
        reply = buffer;
    }

    close(client_fd);

    // Specific to radar
    std::string delimiter = "\n";
    std::string status = reply.substr(
        strlen(command), reply.find(delimiter,strlen(command))-strlen(command));
    
    response->reply = status; // change to response->reply = reply; for full reply

    RCLCPP_INFO(rclcpp::get_logger("nano_tcp_service"), "Sending command: %s", 
        request->command.substr(0,request->command.find(delimiter)).c_str());
    RCLCPP_INFO(rclcpp::get_logger("nano_tcp_service"), "Received: %s", response->reply.c_str());
}

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("send_request_server");

    rclcpp::Service<client_interfaces::srv::SendCommandStringToServer>::SharedPtr service =
        node->create_service<client_interfaces::srv::SendCommandStringToServer>("send_request", &sendRequest);

    RCLCPP_INFO(rclcpp::get_logger("nano_tcp_service"), "Ready to send command");

    rclcpp::spin(node);
    rclcpp::shutdown();
}