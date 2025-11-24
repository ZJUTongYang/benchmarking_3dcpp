#pragma once
#include "robot_model.hpp"

// The beam like robot model: The perception scope is a set of beams. A surface point is captured only if it is within the epsilon distance of the contact point between the surface and a beam
class LineLidar: public RobotModel
{
public: 
    LineLidar(std::string robot_name, 
        int beam_num, double max_distance, double pitch, double raw_range, double epsilon);

    bool isPointCovered(const SurfacePoint& point,
                           const RobotWaypoint& waypoint) const override;

private: 
    int beam_num_;
    double max_distance_;

    double pitch_;
    double yaw_range_;

    double epsilon_;

    std::vector<std::vector<double>> beam_directions_; // [beam_num][2] = [yaw, pitch]
};
