#pragma once

#include <rclcpp/rclcpp.hpp>
#include <H5Cpp.h>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

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

    // Subscription to trigger loading and visualization
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_viz_sub_;

    // Data loaded from HDF5
    std::shared_ptr<std::vector<SurfacePoint>> loaded_surface_points_;
    CoverageResult loaded_result_;
    bool data_is_loaded_ = false;

    // Core functions
    void visualizeLoadedData(); 
    
    // Service callback
    void loadVizCallback(const std_msgs::msg::String::SharedPtr msg);

    // Helper to convert SurfacePoint to PointCloud2
    sensor_msgs::msg::PointCloud2 createPointCloud2Msg(const std::vector<SurfacePoint>& points, const std::vector<int>& point_covered_num);

};
