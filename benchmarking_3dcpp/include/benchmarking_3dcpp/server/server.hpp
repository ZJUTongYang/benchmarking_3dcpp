#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>

#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <benchmarking_3dcpp/robot_model/robot_model.hpp>
#include <benchmarking_3dcpp/scene.hpp>
#include <benchmarking_3dcpp_interfaces/srv/get_coverage_situation.hpp>
#include <benchmarking_3dcpp_interfaces/srv/get_random_surface_points.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>


class Benchmarking3DCPPServer
{
public: 

    Benchmarking3DCPPServer(
        std::shared_ptr<rclcpp::Node> node,
        const std::unordered_map<std::string, std::shared_ptr<Scene> >& scenes,
        const std::unordered_map<std::string, std::shared_ptr<RobotModel> >& robots,
        const std::shared_ptr<CoverageEvaluator> benchmarker):node_(node)
    {
        node_ = node;
        scenes_ = scenes;
        robots_ = robots;
        benchmarker_ = benchmarker;


        coverage_situation_service_ = node_->create_service<benchmarking_3dcpp_interfaces::srv::GetCoverageSituation>(
            "/benchmarking_3dcpp/get_coverage_situation",
            std::bind(&Benchmarking3DCPPServer::handleGetCoverageSituation, this,
            std::placeholders::_1, std::placeholders::_2));
            
        random_surface_points_service_ = node_->create_service<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints>(
            "/benchmarking_3dcpp/get_random_surface_points",
            std::bind(&Benchmarking3DCPPServer::handleGetRandomSurfacePoints, this,
            std::placeholders::_1, std::placeholders::_2));
    }

private: 
    // The same node as the benchmarking platform
    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, std::shared_ptr<Scene> > scenes_;
    std::unordered_map<std::string, std::shared_ptr<RobotModel> > robots_;
    std::shared_ptr<CoverageEvaluator> benchmarker_;

    rclcpp::Service<benchmarking_3dcpp_interfaces::srv::GetCoverageSituation>::SharedPtr coverage_situation_service_;
    rclcpp::Service<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints>::SharedPtr random_surface_points_service_;
  
    void handleGetCoverageSituation(
        const std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetCoverageSituation::Request> request,
        std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetCoverageSituation::Response> response);
    
    void handleGetRandomSurfacePoints(
        const std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints::Request> request,
        std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints::Response> response);

    // Internal implementation
    std::vector<std::vector<int> > getCoverageSituation(const std::string& robot_name, std::string scene_name, 
        const std::vector<Eigen::Vector3d>& position,
        const std::vector<Eigen::Quaterniond>& orientation);

    std::vector<SurfacePoint> getRandomSurfacePoints(const std::string& scene_name, int requested_num);

};


