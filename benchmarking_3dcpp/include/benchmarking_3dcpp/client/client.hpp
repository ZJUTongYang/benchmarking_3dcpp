#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <concepts>

#include <benchmarking_3dcpp/server/server.hpp>
#include <benchmarking_3dcpp/coverage_types.hpp>
#include <nav_msgs/msg/path.hpp>

template<typename T>
concept PathLike = requires(T p){
    {p.size()} -> std::convertible_to<size_t>;
    {p.begin()} -> std::input_iterator;
    {*p.begin()} -> std::convertible_to<geometry_msgs::msg::Pose>;
};

/**
 * @class Benchmarking3DCPPClient
 * @brief An interface that other algorithm can instantiate to re-use functions in the platform
 */
class Benchmarking3DCPPClient: public rclcpp::Node
{
public:

    virtual ~Benchmarking3DCPPClient() = default;

    template<PathLike PathType>
    std::vector<std::vector<int>> getCoverablePointsForEachWaypoint(const std::string& robot_name, 
                            const std::string& scene_name,
                            const PathType& robot_path)
    {
        std::vector<geometry_msgs::msg::Pose> poses;
        if constexpr (std::is_same_v<PathType, nav_msgs::msg::Path>)
        {
            for(const auto& pose_stamped: robot_path.poses)
            {
                poses.emplace_back(pose_stamped.pose);
            }
        }
        else if constexpr (std::is_same_v<PathType, std::vector<geometry_msgs::msg::Pose>>) 
        {
            // if the input has been std::vector<geometry_msgs::msg::Pose>
            poses = robot_path;
        }
        else
        {
            // Assume that we can get Pose from PathType
            static_assert(sizeof(PathType) == 0, "Unsupported path type provided to getCoverablePointsForEachWaypoint. Only nav_msgs::msg::Path and std::vector<geometry_msgs::msg::Pose> are supported.");        
        }

        return getCoverablePointsForEachWaypoint(robot_name, scene_name, poses);
    }

    // returns: a vector of size "path length". Each (the i-th) element is the vector of covered surface points indices (by the i-th waypoint)
    virtual std::vector<std::vector<int>> getCoverablePointsForEachWaypoint(const std::string& robot_name, 
                            const std::string& scene_name,
                            const std::vector<geometry_msgs::msg::Pose>& robot_path) = 0;

    virtual std::vector<geometry_msgs::msg::Point> getRandomSurfacePoints(const std::string& scene_name, int requested_num) = 0;
    
    static std::shared_ptr<Benchmarking3DCPPClient> create();
    
protected:
    Benchmarking3DCPPClient(): Node("benchmarking_3dcpp_client")
    {

    }
    
private:
    // 禁用拷贝构造和赋值
    Benchmarking3DCPPClient(const Benchmarking3DCPPClient&) = delete;
    Benchmarking3DCPPClient& operator=(const Benchmarking3DCPPClient&) = delete;
    
};
