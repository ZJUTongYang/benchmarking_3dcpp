#pragma once

#include <H5Cpp.h>
#include <string>
#include <vector>
#include <benchmarking_3dcpp/types.hpp>

void loadMetaData(H5::H5File& file, std::string& robot_name, 
    std::string& scene_name, std::string& alg_name);

void saveMetaData(H5::H5File& file, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name);

void loadRobotPath(H5::H5File& file, std::vector<RobotWaypoint>& robot_path);

void saveRobotPath(H5::H5File& file, const std::vector<RobotWaypoint>& robot_path);

void loadSurfacePointData(H5::H5File& file, std::vector<SurfacePoint>& surface_points);

void saveSurfacePointData(H5::H5File& file, const std::vector<SurfacePoint>& surface_points);

void saveCoverageIndices(H5::H5File& file, const std::vector<std::vector<int> >& the_coverage_indices);

void loadCoverageResult(H5::H5File& file, CoverageResult& the_result);

void saveCoverageResult(H5::H5File& file, const CoverageResult& the_result);

bool loadFromHDF5(const std::string& filename, 
    std::string& robot_name, std::string& scene_name, std::string& alg_name,
    std::shared_ptr<std::vector<SurfacePoint>>& surface_points,
    CoverageResult& the_result);

bool saveToHDF5(const std::string& filename, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name, 
    const std::shared_ptr<std::vector<SurfacePoint> >& surface_points,
    const std::vector<std::vector<int> >& the_coverage_indices,
    const CoverageResult& the_result);

