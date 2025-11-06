#pragma once

#include <string>
#include <benchmarking_3dcpp/types.hpp>

// Different robots have different motion, perception, and collision models. 
// We should implement these all in this class
class RobotModel
{
public:
    RobotModel(std::string robot_name)
    {
        robot_name_ = robot_name;
    }

    virtual std::string getName() const { return robot_name_; }

    virtual bool isPointCovered(const SurfacePoint& point, 
                               const RobotWaypoint& waypoint) const = 0;

private: 
    std::string robot_name_;
};

