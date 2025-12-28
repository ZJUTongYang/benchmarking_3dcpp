#pragma once

#include <memory>
#include <benchmarking_3dcpp/geometry_types.hpp>
#include <benchmarking_3dcpp/coverage_types.hpp>

class Scene
{
public:
    Scene(std::shared_ptr<GeometryData> scene_object, 
        std::shared_ptr<std::vector<SurfacePoint> > surface_points)
    {
        scene_object_ = scene_object;
        surface_points_ = surface_points;
    }

    std::shared_ptr<GeometryData> scene_object_;
    std::shared_ptr<std::vector<SurfacePoint> > surface_points_;

};