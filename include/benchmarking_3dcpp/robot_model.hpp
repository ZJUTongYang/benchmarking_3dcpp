#pragma once

#include <string>

// Different robots have different motion, perception, and collision models. 
// We should implement these all in this class
class RobotModel
{
public:
    RobotModel(std::string robot_name)
    {
        robot_name_ = robot_name;
    }

    // virtual bool isPointCovered(const SurfacePoint& point, 
    //                            const RobotWaypoint& waypoint, 
    //                            double max_distance) = 0;

private: 
    std::string robot_name_;
};