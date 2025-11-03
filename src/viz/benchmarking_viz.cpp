#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/viz/benchmarking_viz.hpp>


void saveMetaData(H5::H5File& file, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name)
{
    // We need to save metadata
    H5::DataSpace attr_dataspace(H5S_SCALAR);
    H5::StrType str_type(H5::PredType::C_S1, 256);
    
    H5::Attribute robot_attr = file.createAttribute("robot_name", str_type, attr_dataspace);
        robot_attr.write(str_type, robot_name);
        
    H5::Attribute surface_attr = file.createAttribute("surface_name", str_type, attr_dataspace);
        surface_attr.write(str_type, scene_name);

    H5::Attribute alg_attr = file.createAttribute("algorithm_name", str_type, attr_dataspace);
        alg_attr.write(str_type, alg_name);
}

void saveRobotPath(H5::H5File& file, const std::vector<RobotWaypoint>& robot_path)
{
    hsize_t dims[2] = {robot_path.size(), 7};

    H5::DataSpace dataspace(2, dims);

    H5::FloatType datatype(H5::PredType::NATIVE_DOUBLE);

    H5::DataSet dataset = file.createDataSet("/robot_path", datatype, dataspace);

    std::vector<double> data;
    data.resize(robot_path.size() * 7);

    for (size_t i = 0; i < robot_path.size(); i++)
    {
        data[i * 7 + 0] = robot_path[i].position.x();
        data[i * 7 + 1] = robot_path[i].position.y();
        data[i * 7 + 2] = robot_path[i].position.z();
        data[i * 7 + 3] = robot_path[i].orientation.x();
        data[i * 7 + 4] = robot_path[i].orientation.y();
        data[i * 7 + 5] = robot_path[i].orientation.z();
        data[i * 7 + 6] = robot_path[i].orientation.w();
    }
    dataset.write(data.data(), datatype);
}

void saveSurfacePointData(H5::H5File& file, const std::vector<SurfacePoint>& surface_points)
{
    hsize_t dims[2] = {surface_points.size(), 3};

    H5::DataSpace dataspace(2, dims);

    H5::FloatType datatype(H5::PredType::NATIVE_DOUBLE);

    H5::DataSet dataset = file.createDataSet("/surface_points", datatype, dataspace);

    std::vector<double> data;
    data.resize(surface_points.size() * 3);

    for (size_t i = 0; i < surface_points.size(); i++)
    {

        data[i * 3 + 0] = surface_points[i].position.x();
        data[i * 3 + 1] = surface_points[i].position.y();
        data[i * 3 + 2] = surface_points[i].position.z();
    }

    dataset.write(data.data(), datatype);

}

void saveCoverageResult(H5::H5File& file, const CoverageResult& the_result)
{
    H5::Group coverage_group = file.createGroup("/coverage_result");

    H5::DataSpace scalar_dataspace(H5S_SCALAR);

    H5::Attribute coverage_attr = coverage_group.createAttribute("coverage_ratio", 
                                                                H5::PredType::NATIVE_DOUBLE, 
                                                                scalar_dataspace);
    coverage_attr.write(H5::PredType::NATIVE_DOUBLE, &the_result.coverage_ratio);


    H5::Attribute time_attr = coverage_group.createAttribute("computation_time", 
                                                            H5::PredType::NATIVE_DOUBLE, 
                                                            scalar_dataspace);
    time_attr.write(H5::PredType::NATIVE_DOUBLE, &the_result.computation_time);

    H5::Attribute total_attr = coverage_group.createAttribute("total_points", 
                                                                H5::PredType::NATIVE_HSIZE, 
                                                                scalar_dataspace);
    hsize_t total_points = the_result.total_points;
    total_attr.write(H5::PredType::NATIVE_HSIZE, &total_points);


    H5::Attribute covered_attr = coverage_group.createAttribute("covered_points", 
                                                                H5::PredType::NATIVE_HSIZE, 
                                                                scalar_dataspace);
    hsize_t covered_points = the_result.covered_points;
    covered_attr.write(H5::PredType::NATIVE_HSIZE, &covered_points);

    // save the number of times that each surface point is continuously covered
    if (!the_result.point_covered_num.empty()) 
    {
        hsize_t dims[1] = {the_result.point_covered_num.size()};
        H5::DataSpace dataspace(1, dims);
        
        H5::DataSet coverage_data = coverage_group.createDataSet("point_covered_num",
                                                                H5::PredType::NATIVE_INT,
                                                                dataspace);
        coverage_data.write(the_result.point_covered_num.data(),
                          H5::PredType::NATIVE_INT);
    }

}

void saveCoverageMask(H5::Group& group, const std::vector<bool>& coverage_mask)
{
    if(coverage_mask.empty())
        return ;

    hsize_t dims[1] = {coverage_mask.size()};
    H5::DataSpace dataspace(1, dims);
    
    // 将bool转换为unsigned char保存，因为HDF5没有原生bool类型
    H5::IntType datatype(H5::PredType::NATIVE_UCHAR);
    H5::DataSet dataset = group.createDataSet("coverage_mask", datatype, dataspace);


    std::vector<unsigned char> mask_data(coverage_mask.size());
    for (size_t i = 0; i < coverage_mask.size(); i++) {
        mask_data[i] = coverage_mask[i] ? 1 : 0;
    }

    dataset.write(mask_data.data(), H5::PredType::NATIVE_UCHAR);
    
    H5::DataSpace attr_dataspace(H5S_SCALAR);
    H5::StrType str_type(H5::PredType::C_S1, 256);
    
    H5::Attribute desc_attr = dataset.createAttribute("description", str_type, attr_dataspace);
    std::string description = "Coverage mask: 1=covered, 0=not covered";
    desc_attr.write(str_type, description);
}

bool saveToHDF5(const std::string& filename, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name, 
    const std::shared_ptr<std::vector<SurfacePoint> >& surface_points,
    const CoverageResult& the_result)
{
    try
    {
        H5::H5File file(filename, H5F_ACC_TRUNC);
        saveMetaData(file, robot_name, scene_name, alg_name);
        saveRobotPath(file, the_result.robot_path);
        saveSurfacePointData(file, *surface_points);
        saveCoverageResult(file, the_result);
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

BenchmarkingViz::BenchmarkingViz():Node("benchmarking_viz_node")
{
    // // Publishers
    // coverage_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //     "coverage_visualization", 10);

    // path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    //     "path_visualization", 10
    // );

    // scene_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    //     "scene_visualization", 10
    // );
}


// void Benchmarking3DCPP::pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
// {
//     std::vector<RobotWaypoint> path;
//     path.reserve(msg->poses.size());
    
//     for (const auto& pose : msg->poses) {
//         RobotWaypoint wp;
//         wp.position = Eigen::Vector3d(
//             pose.position.x, pose.position.y, pose.position.z);
//         wp.orientation = Eigen::Quaterniond(
//             pose.orientation.w, pose.orientation.x,
//             pose.orientation.y, pose.orientation.z);
//         wp.coverage_radius = 0.05; // Default radius
//         wp.timestamp = this->now().seconds();
//         path.push_back(wp);
//     }
    
//     // Calculate coverage    
//     benchmarker_->calculateCoverage(curr_test_index_);
    
//     RCLCPP_INFO(this->get_logger(), "Coverage: %.2f%% (%zu/%zu points)",
//             benchmarker_->getTask(curr_test_index_).result.coverage_ratio * 100.0, 
//             benchmarker_->getTask(curr_test_index_).result.covered_points, 
//             benchmarker_->getTask(curr_test_index_).result.total_points);
            
//     publishVisualization(benchmarker_->getTask(curr_test_index_));

//     // We deal with the next test
//     curr_test_index_++;
// }

// void Benchmarking3DCPP::publishVisualization(const Task& the_task) 
// {
//     const auto& result = the_task.result;

//     visualization_msgs::msg::MarkerArray markers;
    
//     // Create marker for covered points (green)
//     visualization_msgs::msg::Marker covered_marker;
//     covered_marker.header.frame_id = "world";
//     covered_marker.header.stamp = this->now();
//     covered_marker.ns = "coverage";
//     covered_marker.id = 0;
//     covered_marker.type = visualization_msgs::msg::Marker::POINTS;
//     covered_marker.action = visualization_msgs::msg::Marker::ADD;
//     covered_marker.pose.orientation.w = 1.0;
//     covered_marker.scale.x = 0.005;
//     covered_marker.scale.y = 0.005;
//     covered_marker.color.g = 1.0;
//     covered_marker.color.a = 1.0;
    
//     // Create marker for uncovered points (red)
//     visualization_msgs::msg::Marker uncovered_marker = covered_marker;
//     uncovered_marker.id = 1;
//     uncovered_marker.color.r = 1.0;
//     uncovered_marker.color.g = 0.0;
    
//     // TODO: Populate markers with actual point positions
//     for(size_t i = 0; i < result.total_points; ++i)
//     {
//         geometry_msgs::msg::Point point;
//         if(result.coverage_mask[i] == true)
//         {
//             point.x = result.surface_points[i].position[0];
//             point.y = result.surface_points[i].position[1];
//             point.z = result.surface_points[i].position[2];
//             covered_marker.points.push_back(point);
//         }
//         else
//         {
//             point.x = result.surface_points[i].position[0];
//             point.y = result.surface_points[i].position[1];
//             point.z = result.surface_points[i].position[2];
//             uncovered_marker.points.push_back(point);
//         }

//     }
    
//     markers.markers.push_back(covered_marker);
//     markers.markers.push_back(uncovered_marker);
    
//     coverage_pub_->publish(markers);

//     // Add robot path visualization if available
//     if (!result.robot_path.empty()) {
//         visualization_msgs::msg::Marker path_marker;
//         path_marker.header.frame_id = "world";
//         path_marker.header.stamp = this->now();
//         path_marker.ns = "robot_path";
//         path_marker.id = 3;
//         path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//         path_marker.action = visualization_msgs::msg::Marker::ADD;
//         path_marker.pose.orientation.w = 1.0;
//         path_marker.scale.x = 0.01;
//         path_marker.color.b = 1.0;
//         path_marker.color.g = 0.5;
//         path_marker.color.a = 1.0;

//         visualization_msgs::msg::Marker waypoint_marker;
//         waypoint_marker.header.frame_id = "world";
//         waypoint_marker.header.stamp = this->now();
//         waypoint_marker.ns = "robot_waypoints";
//         waypoint_marker.id = 4;
//         waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
//         waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
//         waypoint_marker.pose.orientation.w = 1.0;
//         waypoint_marker.scale.x = 0.02;
//         waypoint_marker.scale.y = 0.02;
//         waypoint_marker.scale.z = 0.02;
//         waypoint_marker.color.r = 1.0;
//         waypoint_marker.color.g = 1.0;
//         waypoint_marker.color.a = 0.8;

//         // for (const auto& waypoint : result.robot_path) {
//         //     geometry_msgs::msg::Point path_point;
//         //     path_point.x = waypoint.position.x();
//         //     path_point.y = waypoint.position.y();
//         //     path_point.z = waypoint.position.z();
            
//         //     path_marker.points.push_back(path_point);
//         //     waypoint_marker.points.push_back(path_point);
//         // }
//         for(size_t i = 0; i < result.robot_path.size()-1; ++i)
//         {
//             geometry_msgs::msg::Point path_point;
//             path_point.x = result.robot_path[i].position.x();
//             path_point.y = result.robot_path[i].position.y();
//             path_point.z = result.robot_path[i].position.z();
//             path_marker.points.push_back(path_point);

//             path_point.x = result.robot_path[i+1].position.x();
//             path_point.y = result.robot_path[i+1].position.y();
//             path_point.z = result.robot_path[i+1].position.z();
//             path_marker.points.push_back(path_point);
//         }
//         // We re-add the last point
//         {
//             geometry_msgs::msg::Point path_point;
//             path_point.x = result.robot_path[result.robot_path.size()-1].position.x();
//             path_point.y = result.robot_path[result.robot_path.size()-1].position.y();
//             path_point.z = result.robot_path[result.robot_path.size()-1].position.z();
//             path_marker.points.push_back(path_point);
//         }


//         // if (!path_marker.points.empty()) {
//         //     markers.markers.push_back(path_marker);
//         // }
//         // if (!waypoint_marker.points.empty()) {
//         //     markers.markers.push_back(waypoint_marker);
//         // }

//         path_pub_->publish(path_marker);
//     }

//     // We publish the triangular mesh
//     // Add mesh wireframe visualization
//     std::string scene_name = the_task.scene.name;
//     const std::shared_ptr<GeometryData> p_the_scene = scenes_[scene_name]->scene_object_;
//     // If the scene is a mesh, we visualize it
//     const auto* mesh_data = static_cast<const TriangleMeshData*>(p_the_scene.get());


//     const auto& vertices = mesh_data->getData()->vertices_;
//     const auto& triangles = mesh_data->getData()->triangles_;
//     visualization_msgs::msg::Marker mesh_marker;
//     for (const auto& triangle : triangles) {
//         if (static_cast<size_t>(triangle[0]) < vertices.size() &&
//             static_cast<size_t>(triangle[1]) < vertices.size() && 
//             static_cast<size_t>(triangle[2]) < vertices.size()) {
//             // Add three edges for each triangle
//             geometry_msgs::msg::Point p1, p2, p3;
//             p1.x = vertices[triangle[0]].x(); p1.y = vertices[triangle[0]].y(); p1.z = vertices[triangle[0]].z();
//             p2.x = vertices[triangle[1]].x(); p2.y = vertices[triangle[1]].y(); p2.z = vertices[triangle[1]].z();
//             p3.x = vertices[triangle[2]].x(); p3.y = vertices[triangle[2]].y(); p3.z = vertices[triangle[2]].z();
            
//             // Edge 1-2
//             mesh_marker.points.push_back(p1);
//             mesh_marker.points.push_back(p2);
//             // Edge 2-3
//             mesh_marker.points.push_back(p2);
//             mesh_marker.points.push_back(p3);
//             // Edge 3-1
//             mesh_marker.points.push_back(p3);
//             mesh_marker.points.push_back(p1);
//         }
//     }
//     mesh_marker.header.frame_id = "world";
//     mesh_marker.header.stamp = this->now();
//     mesh_marker.ns = "mesh_wireframe";
//     mesh_marker.id = 2;
//     mesh_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
//     mesh_marker.action = visualization_msgs::msg::Marker::ADD;
//     mesh_marker.pose.orientation.w = 1.0;
//     mesh_marker.scale.x = 0.002;  // Thin lines
//     mesh_marker.color.r = 0.5;
//     mesh_marker.color.g = 0.5;
//     mesh_marker.color.b = 0.5;
//     mesh_marker.color.a = 0.3;    // Semi-transparent
//     scene_pub_->publish(mesh_marker);


// }
