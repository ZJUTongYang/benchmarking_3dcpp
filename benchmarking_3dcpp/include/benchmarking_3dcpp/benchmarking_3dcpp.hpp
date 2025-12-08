#pragma once

#include <rclcpp/rclcpp.hpp>
#include <open3d/Open3D.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <benchmarking_3dcpp/types.hpp>
#include <benchmarking_3dcpp/robot_model/robot_model.hpp>
#include <benchmarking_3dcpp/alg/coverage_algorithm.hpp>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <memory>
#include <nuc_msgs/srv/get_nuc.hpp>
#include <optional>
#include <benchmarking_3dcpp/input_types.hpp>
#include <yaml-cpp/yaml.h>
#include <benchmarking_3dcpp/scene.hpp>

class Benchmarking3DCPP: public rclcpp::Node
{
public:
    Benchmarking3DCPP();

private:
    void initialize();

    void scheduleAllTests(int num_robots, int num_scenes, int num_algorithms, const YAML::Node& config);
    void runSingleTest();

    void saveEvalToFile(int task_id);

    std::shared_ptr<GeometryData> loadGeometryFile(const std::string& filename)
    {
        return geometryLoader_->load(filename);
    }

    std::unordered_map<std::string, std::shared_ptr<RobotModel> > robots_;
    std::unordered_map<std::string, std::shared_ptr<Scene> > scenes_;
    std::unordered_map<std::string, std::shared_ptr<CoverageAlgorithm> > algorithms_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scene_pub_;
    
    bool test_is_running_;
    int curr_test_index_;
    bool initialized_;
    bool use_cuda_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<CoverageEvaluator> benchmarker_;

    std::unique_ptr<GeometryLoader> geometryLoader_;
    
    std::unique_ptr<SurfaceSampler> surface_sampler_;
    
    std::string config_filename_;
};
