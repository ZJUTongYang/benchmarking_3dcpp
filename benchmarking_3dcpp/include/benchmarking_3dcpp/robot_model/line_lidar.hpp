#pragma once
#include "robot_model.hpp"

// The beam like robot model: The perception scope is a set of beams. A surface point is captured only if it is within the epsilon distance of the contact point between the surface and a beam
class LineLidar: public RobotModel
{
public: 
    LineLidar(std::string robot_name, 
        unsigned int beam_num, double max_distance, double pitch, double raw_range, double epsilon);

    bool isPointCovered(const SurfacePoint& point,
                           const RobotWaypoint& waypoint) const override;

    double getEpsilon() const
    {
        return epsilon_;
    }

    int getBeamNum() const
    {
        return beam_num_;
    }

    double getMaxDistance() const
    {
        return max_distance_;
    }

    void getBeamsTripleArray(float* array) const
    {
        for (size_t i = 0; i < beam_num_; ++i) 
        {
            array[3 * i] = static_cast<float>(beam_directions_[i][0]);
            array[3 * i + 1] = static_cast<float>(beam_directions_[i][1]);
            array[3 * i + 2] = static_cast<float>(beam_directions_[i][2]);
        }
    }
    
private: 
    unsigned int beam_num_;
    double max_distance_;

    double pitch_;
    double yaw_range_;

    double epsilon_;

    std::vector<std::vector<double>> beam_directions_; // [beam_num][2] = [yaw, pitch]
};
