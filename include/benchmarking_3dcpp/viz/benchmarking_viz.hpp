#pragma once

#include <rclcpp/rclcpp.hpp>
#include <H5Cpp.h>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class BenchmarkingViz : public rclcpp::Node
{
public: 
    BenchmarkingViz();

private: 
    // Publish the coloured point cloud 
    // Idealy, regardless of the format of the surface, we have sampled it, 
    // so we can visualize it as the point cloud
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_pub_;

    // We publish the robot path (which should be a tf strip)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    // Service to trigger loading and visualization
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_viz_service_;

    // Data loaded from HDF5
    std::shared_ptr<std::vector<SurfacePoint>> loaded_surface_points_;
    CoverageResult loaded_result_;
    bool data_is_loaded_ = false;

    // Core functions
    void visualizeLoadedData(); // 新的发布函数
    void loadAndVisualize(const std::string& filename); // 加载并准备发布的函数
    

    // Service callback
    void handleLoadVizService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // Helper to convert SurfacePoint to PointCloud2
    sensor_msgs::msg::PointCloud2 createPointCloud2Msg(const std::vector<SurfacePoint>& points, const std::vector<int>& point_covered_num);

  
    // rclcpp::TimerBase::SharedPtr timer_;

};
