#pragma once

#include <rclcpp/rclcpp.hpp>
#include <H5Cpp.h>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>

void saveMetaData(H5::H5File& file, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name);

void saveRobotPath(H5::H5File& file, const std::vector<RobotWaypoint>& robot_path);

void saveSurfacePointData(H5::H5File& file, const std::vector<SurfacePoint>& surface_points);

void saveCoverageInfo(H5::H5File& file);

bool saveToHDF5(const std::string& filename, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name, 
    const std::shared_ptr<std::vector<SurfacePoint> >& surface_points,
    const CoverageResult& the_result);

class BenchmarkingViz : public rclcpp::Node
{
public: 
    BenchmarkingViz();

};