#include <rclcpp/rclcpp.hpp>
#include <benchmarking_3dcpp/server/server.hpp>
#include <benchmarking_3dcpp/types.hpp>
#include <unordered_set>

void Benchmarking3DCPPServer::handleGetCoverageSituation(
    const std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetCoverageSituation::Request> request,
    std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetCoverageSituation::Response> response)
{
    
    try {
        // Convert ROS messages to Eigen vectors
        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Quaterniond> orientations;
        
        for (const auto& point : request->robot_path.poses) 
        {
            positions.emplace_back(point.pose.position.x, point.pose.position.y, point.pose.position.z);
            orientations.emplace_back(point.pose.orientation.w, point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z);
        }
        
        // Get coverage indices using the internal function
        std::vector<std::vector<int>> coverage_indices = getCoverageSituation(
            request->robot_name, request->scene_name, positions, orientations);
        
        // Flatten the 2D array and compute start indices
        response->covered_surface_points.clear();
        response->start_indices.clear();
        
        int current_start = 0;
        for (const auto& row : coverage_indices) {
            // Add the start index for this row
            response->start_indices.push_back(current_start);
            
            // Add all elements of this row to the flattened array
            for (int idx : row) {
                response->covered_surface_points.push_back(idx);
            }
            
            // Update the start index for the next row
            current_start += row.size();
        }
        
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error in handleGetCoverageSituation: %s", e.what());
        // Set empty response to indicate failure
        response->covered_surface_points.clear();
        response->start_indices.clear();
    }
}

void Benchmarking3DCPPServer::handleGetRandomSurfacePoints(
    const std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints::Request> request,
    std::shared_ptr<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints::Response> response)
{
    try{
        // Get random surface points using the internal function
        std::vector<SurfacePoint> surface_points = getRandomSurfacePoints(
            request->scene_name, request->num_points);
        std::cout << "YT: we check size of the vector: " << surface_points.size() << std::endl;

        // Convert SurfacePoint to geometry_msgs::msg::Point
        std::vector<geometry_msgs::msg::Point> points;
        points.reserve(surface_points.size());
        for (const auto& sp : surface_points) {
            geometry_msgs::msg::Point p;
            p.x = sp.position.x();
            p.y = sp.position.y();
            p.z = sp.position.z();
            points.push_back(p);
        }
        
        // Copy the points to the response
        response->surface_points = points;
    
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error in handleGetRandomSurfacePoints: %s", e.what());
        // Set empty response to indicate failure
        response->surface_points.clear();
    }
}

std::vector<std::vector<int> > Benchmarking3DCPPServer::getCoverageSituation(const std::string& robot_name, std::string scene_name, 
    const std::vector<Eigen::Vector3d>& position, const std::vector<Eigen::Quaterniond>& orientation)
{
    std::vector<std::vector<int> > coverage_indices;

    if(position.size() != orientation.size())
    {
        throw std::runtime_error("Position and orientation vectors must have the same size");
    }

    // find the robot
    auto robot_it = robots_.find(robot_name);
    if (robot_it == robots_.end()) {
        throw std::runtime_error("Robot '" + robot_name + "' not found");
    }
    auto p_robot_model = robot_it->second;
    
    // find the scene
    auto scene_it = scenes_.find(scene_name);
    if (scene_it == scenes_.end()) {
        throw std::runtime_error("Scene '" + scene_name + "' not found");
    }
    auto& scene = scene_it->second;
    std::vector<SurfacePoint>& surface_points = *(scene->surface_points_);


    std::vector<RobotWaypoint> path(position.size());
    for(size_t i = 0; i < position.size(); i++)
    {
        path[i].position = position[i];
        path[i].orientation = orientation[i];
    }
    
    if (benchmarker_) {
        coverage_indices = benchmarker_->evaluateCoverage(p_robot_model, surface_points, path);
    } else {
        throw std::runtime_error("Coverage evaluator not initialized");
    }

    return coverage_indices;
}

std::vector<SurfacePoint> Benchmarking3DCPPServer::getRandomSurfacePoints(const std::string& scene_name, int requested_num)
{
    std::vector<SurfacePoint> result;

    auto scene_it = scenes_.find(scene_name);
    if (scene_it == scenes_.end()) {
        throw std::runtime_error("Scene '" + scene_name + "' not found");
    }
    auto& scene = scene_it->second;
    std::vector<SurfacePoint>& surface_points = *(scene->surface_points_);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, surface_points.size() - 1); // 均匀分布
    
    // 避免重复选择
    // To avoid repeated selection
    std::unordered_set<int> selected_indices; 

    while (rclcpp::ok() && selected_indices.size() < static_cast<size_t>(requested_num)) 
    {
        int idx = distrib(gen);
        if (selected_indices.find(idx) == selected_indices.end()) 
        {
            selected_indices.insert(idx);
            result.push_back(surface_points[idx]);
        }
    }

    return result;
}
