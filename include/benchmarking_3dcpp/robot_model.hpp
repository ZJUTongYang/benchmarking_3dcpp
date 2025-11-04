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

    virtual bool isPointCovered(const SurfacePoint& point, 
                               const RobotWaypoint& waypoint) const = 0;

private: 
    std::string robot_name_;
};

// The circular tool robot model: The projective shape of the tool model on the tangent space of the suface is a circle of radius r
// The maximum intrusion distance of the tool is d
class CircularTool: public RobotModel
{
public: 
    CircularTool(std::string robot_name, double r, double d):RobotModel(robot_name), R_(r), D_(d)
    {}

    bool isPointCovered(const SurfacePoint& point, 
                           const RobotWaypoint& waypoint) const override;

    float getRadius() const { return static_cast<float>(R_); }
    float getDepth() const { return static_cast<float>(D_); }
private: 
    double R_;
    double D_;
};

// The beam like robot model: The perception scope is a set of beams. A surface point is captured only if it is within the epsilon distance of the contact point between the surface and a beam
class BeamLike: public RobotModel
{
public: 
    BeamLike(std::string robot_name, std::vector<std::pair<double, double> >& beam_orientation, double epsilon):
        RobotModel(robot_name), beam_orientation_(beam_orientation), epsilon_(epsilon)
    {}

    bool isPointCovered(const SurfacePoint& point,
                           const RobotWaypoint& waypoint) const override;

private: 
    std::vector<std::pair<double, double> > beam_orientation_;
    double epsilon_;

};