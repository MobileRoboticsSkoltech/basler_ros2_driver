//
// Created by devilox on 8/24/21.
//
//-----------------------------//
#include <signal.h>
#include <iostream>
//-----------------------------//
#include "basler_ros2_driver/BaslerNode.h"
//-----------------------------//
void sigintHandler(int tSigNum) {
    std::cout << "Receive signum: " << tSigNum << std::endl;
    rclcpp::shutdown();
}
//-----------------------------//
int main(int argc, char ** argv) {
    signal(SIGINT, sigintHandler);

    rclcpp::init(argc, argv);

    auto CameraNode = std::make_shared <BaslerNode>();
    CameraNode -> startLoop();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
