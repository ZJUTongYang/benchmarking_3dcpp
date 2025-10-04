#pragma once

#include <rclcpp/rclcpp.hpp>

class Benchmarking3DCPP: public rclcpp::Node
{
public:
    Benchmarking3DCPP();

    void runBenchmarking();

private:
    void loadPath();
    void loadScene();
    void loadCoveringModel();

    // For the scene
    std::string scene_filename_;

};