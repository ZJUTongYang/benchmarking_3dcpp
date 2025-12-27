#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <benchmarking_3dcpp/server/server.hpp>
#include <benchmarking_3dcpp/types.hpp>


/**
 * @class Benchmarking3DCPPClient
 * @brief An interface that other algorithm can instantiate to re-use functions in the platform
 */
class Benchmarking3DCPPClient: public rclcpp::Node
{
public:

    virtual ~Benchmarking3DCPPClient() = default;


    virtual std::vector<std::vector<int>> getCoverageSituation(const std::string& robot_name, 
                            const std::string& scene_name,
                            const nav_msgs::msg::Path& robot_path) = 0;

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
