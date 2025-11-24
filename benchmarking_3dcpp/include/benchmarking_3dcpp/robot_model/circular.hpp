#pragma once

#include "robot_model.hpp"


// The circular robot model: The projective shape of the tool model on the tangent space of the suface is a circle of radius r
// The maximum intrusion distance of the tool is d
class Circular: public RobotModel
{
public: 
    Circular(std::string robot_name, double r, double d):RobotModel(robot_name), R_(r), D_(d)
    {}

    bool isPointCovered(const SurfacePoint& point, 
                           const RobotWaypoint& waypoint) const override;

    float getRadius() const { return static_cast<float>(R_); }
    float getDepth() const { return static_cast<float>(D_); }
private: 
    double R_;
    double D_;
};

