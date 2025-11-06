#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/viz/benchmarking_viz.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <benchmarking_3dcpp/utils/h5_helper.hpp>

BenchmarkingViz::BenchmarkingViz():Node("benchmarking_viz_node")
{
    // Publishers
    coverage_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "coverage_visualization", 10);

    path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "path_visualization", 10
    );

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "surface_pointcloud", 10
    );

    // Service to trigger loading
    load_viz_service_ = this->create_service<std_srvs::srv::Trigger>(
        "load_and_visualize_h5",
        std::bind(&BenchmarkingViz::handleLoadVizService, this, std::placeholders::_1, std::placeholders::_2));
}


void BenchmarkingViz::handleLoadVizService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // Unused

    // --- 在这里指定你要加载的 HDF5 文件 ---
    // 你可以从服务请求中获取文件名，或者硬编码一个用于测试
    std::string filename = "your_robot_your_scene_your_algorithm.h5"; // <--- 修改为你的文件名
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmarking_3dcpp");
    std::filesystem::path file_fullname = std::filesystem::path(package_share_directory) / "output" / filename;

    RCLCPP_INFO(this->get_logger(), "Attempting to load and visualize data from: %s", file_fullname.c_str());

    if (!std::filesystem::exists(file_fullname)) {
        RCLCPP_ERROR(this->get_logger(), "File not found: %s", file_fullname.c_str());
        response->success = false;
        response->message = "File not found: " + file_fullname.string();
        return;
    }

    std::string robot_name, scene_name, alg_name;
    if (loadFromHDF5(file_fullname.string(), robot_name, scene_name, alg_name, loaded_surface_points_, loaded_result_)) {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded HDF5 file. Robot: %s, Scene: %s, Algorithm: %s", 
            robot_name.c_str(), scene_name.c_str(), alg_name.c_str());
        data_is_loaded_ = true;
        visualizeLoadedData(); // 加载成功后立即发布
        response->success = true;
        response->message = "Successfully loaded and visualized: " + file_fullname.string();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load HDF5 file.");
        response->success = false;
        response->message = "Failed to load HDF5 file: " + file_fullname.string();
    }
}


void BenchmarkingViz::visualizeLoadedData()
{
    if (!data_is_loaded_) {
        RCLCPP_WARN(this->get_logger(), "No data loaded to visualize. Call the service first.");
        return;
    }

    // 1. 发布点云
    sensor_msgs::msg::PointCloud2 cloud_msg = createPointCloud2Msg(*loaded_surface_points_, loaded_result_.point_covered_num);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "world"; // 确保与 RViz 中的 Fixed Frame 一致
    pointcloud_pub_->publish(cloud_msg);

    // // 2. 发布覆盖率标记
    // visualization_msgs::msg::MarkerArray markers;
    
    // visualization_msgs::msg::Marker covered_marker;
    // covered_marker.header.frame_id = "world";
    // covered_marker.header.stamp = this->now();
    // covered_marker.ns = "coverage_points";
    // covered_marker.id = 0;
    // covered_marker.type = visualization_msgs::msg::Marker::POINTS;
    // covered_marker.action = visualization_msgs::msg::Marker::ADD;
    // covered_marker.pose.orientation.w = 1.0;
    // covered_marker.scale.x = 0.005;
    // covered_marker.scale.y = 0.005;
    // covered_marker.color.g = 1.0;
    // covered_marker.color.a = 1.0;
    
    // visualization_msgs::msg::Marker uncovered_marker = covered_marker;
    // uncovered_marker.id = 1;
    // uncovered_marker.color.r = 1.0;
    // uncovered_marker.color.g = 0.0;

    // for(size_t i = 0; i < loaded_surface_points_->size(); ++i)
    // {
    //     geometry_msgs::msg::Point p;
    //     p.x = (*loaded_surface_points_)[i].position.x();
    //     p.y = (*loaded_surface_points_)[i].position.y();
    //     p.z = (*loaded_surface_points_)[i].position.z();

    //     if(loaded_result_.coverage_mask[i])
    //     {
    //         covered_marker.points.push_back(p);
    //     }
    //     else
    //     {
    //         uncovered_marker.points.push_back(p);
    //     }
    // }
    
    // markers.markers.push_back(covered_marker);
    // markers.markers.push_back(uncovered_marker);
    // coverage_pub_->publish(markers);

    // 3. 发布机器人路径
    if (!loaded_result_.robot_path.empty()) {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "world";
        path_marker.header.stamp = this->now();
        path_marker.ns = "robot_path";
        path_marker.id = 2;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = 0.01;
        path_marker.color.b = 1.0;
        path_marker.color.g = 0.5;
        path_marker.color.a = 1.0;

        for (const auto& wp : loaded_result_.robot_path) {
            geometry_msgs::msg::Point p;
            p.x = wp.position.x();
            p.y = wp.position.y();
            p.z = wp.position.z();
            path_marker.points.push_back(p);
        }
        path_pub_->publish(path_marker);
    }
    
    RCLCPP_INFO(this->get_logger(), "Published visualization data.");
}


sensor_msgs::msg::PointCloud2 BenchmarkingViz::createPointCloud2Msg(const std::vector<SurfacePoint>& points, const std::vector<int>& point_covered_num)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;

    // We check the validity of the data
    if (points.empty() || point_covered_num.size() != points.size()) {
        RCLCPP_WARN(this->get_logger(), "Point data or coverage data is empty or mismatched. Returning empty cloud.");
        return cloud_msg; // 返回一个空的云消息
    }

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz", "rgb"); // 添加 xyz 和 rgb 字段
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    // We find the maximally covered num
    int max_coverage = 0;
    if (!point_covered_num.empty()) {
        max_coverage = *std::max_element(point_covered_num.begin(), point_covered_num.end());
    }

    // All points are not covered
    if (max_coverage == 0) {
        for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            *iter_x = points[i].position.x();
            *iter_y = points[i].position.y();
            *iter_z = points[i].position.z();
            *iter_r = 0;
            *iter_g = 0;
            *iter_b = 255; // 全部设为蓝色
        }
        return cloud_msg;
    }

    // loop through all points to set the colour
    for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
        *iter_x = points[i].position.x();
        *iter_y = points[i].position.y();
        *iter_z = points[i].position.z();

        // normalize to [0, 1]
        float normalized_value = static_cast<float>(point_covered_num[i]) / static_cast<float>(max_coverage);

        uint8_t r, g, b;
        b = static_cast<uint8_t>((1.0f - normalized_value) * 255.0f);
        r = static_cast<uint8_t>(normalized_value * 255.0f);
        g = 0; // the green channel

        *iter_r = r;
        *iter_g = g;
        *iter_b = b;
    }

    return cloud_msg;
}
