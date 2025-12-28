#include <benchmarking_3dcpp/client/client.hpp>
#include <memory>
#include <benchmarking_3dcpp/coverage_types.hpp>


class Benchmarking3DCPPClientImpl : public Benchmarking3DCPPClient
{
public: 

    Benchmarking3DCPPClientImpl()
    {
        coverage_situation_client_ = this->create_client<benchmarking_3dcpp_interfaces::srv::GetCoverablePointsForEachWaypoint>(
            "/benchmarking_3dcpp/get_coverable_points_for_each_waypoint");
        random_surface_points_client_ = this->create_client<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints>(
            "/benchmarking_3dcpp/get_random_surface_points");
    }
    std::vector<std::vector<int>> getCoverablePointsForEachWaypoint(const std::string& robot_name, 
                        const std::string& scene_name,
                        const std::vector<geometry_msgs::msg::Pose>& robot_path) override
    {
        std::vector<std::vector<int>> coverage_indices;

        auto request = std::make_shared<benchmarking_3dcpp_interfaces::srv::GetCoverablePointsForEachWaypoint::Request>();
        request->robot_name = robot_name;
        request->scene_name = scene_name;
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        for (const auto& pose : robot_path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "NO_USE"; // reuse header
            pose_stamped.pose = pose;

            path_msg.poses.push_back(pose_stamped);
        }
        request->robot_path = path_msg;
        
        // Send the request with callback
        std::promise<benchmarking_3dcpp_interfaces::srv::GetCoverablePointsForEachWaypoint::Response::SharedPtr> promise;
        auto future = promise.get_future();
        
        auto callback = [&promise](rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetCoverablePointsForEachWaypoint>::SharedFuture response_future) {
            promise.set_value(response_future.get());
        };
        
        coverage_situation_client_->async_send_request(request, callback);
        
        // Wait for the response
        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for get_coverage_situation service");
            return coverage_indices;
        }
        
        auto response = future.get();
        // Convert response back to coverage_indices
        coverage_indices.clear();
        
        // get the flattened data and the indices
        const std::vector<int>& flat_data = response->covered_surface_points;
        const std::vector<int>& start_indices = response->start_indices;
        
        // reconstruct the two-dim array
        for (size_t i = 0; i < start_indices.size(); ++i)
        {
            std::vector<int> row;
            
            // 当前行的起始位置
            size_t start = start_indices[i];
            
            // 下一行的起始位置（如果是最后一行，则使用数据的总长度）
            size_t end = (i + 1 < start_indices.size()) ? start_indices[i + 1] : flat_data.size();
            
            // 提取当前行的数据
            for (size_t j = start; j < end; ++j)
            {
                if (j < flat_data.size())
                {
                    row.emplace_back(flat_data[j]);
                }
            }
            
            coverage_indices.emplace_back(row);
        }
        
        return coverage_indices;
    }

    std::vector<geometry_msgs::msg::Point> getRandomSurfacePoints(const std::string& scene_name, int requested_num) override
    {
        std::vector<geometry_msgs::msg::Point> result;

        auto request = std::make_shared<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints::Request>();
        request->scene_name = scene_name;
        request->num_points = requested_num;
        
        // Send the request with callback
        std::promise<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints::Response::SharedPtr> promise;
        auto future = promise.get_future();
        
        auto callback = [&promise](rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints>::SharedFuture response_future) {
            promise.set_value(response_future.get());
        };
        
        random_surface_points_client_->async_send_request(request, callback);
        
        // Wait for the response
        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for get_random_surface_points service");
            return result;
        }
        
        auto response = future.get();
        // Convert response back to SurfacePoint vector
        result.clear();
        for (const auto& point : response->surface_points)
        {
            SurfacePoint sp;
            geometry_msgs::msg::Point point_temp;
            point_temp.x = point.x;
            point_temp.y = point.y;
            point_temp.z = point.z;
            result.push_back(point_temp);
        }
        
        return result;
    }
private: 

    rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetCoverablePointsForEachWaypoint>::SharedPtr coverage_situation_client_;
    rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetRandomSurfacePoints>::SharedPtr random_surface_points_client_;

    Benchmarking3DCPPClientImpl(const Benchmarking3DCPPClient&) = delete;
    Benchmarking3DCPPClientImpl& operator=(const Benchmarking3DCPPClientImpl&) = delete;
};

std::shared_ptr<Benchmarking3DCPPClient> Benchmarking3DCPPClient::create()
{
    return std::make_shared<Benchmarking3DCPPClientImpl>();
}
