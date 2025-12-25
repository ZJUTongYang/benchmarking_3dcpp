#pragma once
#include <benchmarking_3dcpp/common_types.hpp>
#include <benchmarking_3dcpp/eval/surface_sampler.hpp>
#include <benchmarking_3dcpp/input_types.hpp>
#include <benchmarking_3dcpp/types.hpp>
// #include <open3d/geometry/TriangleMesh.h>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <benchmarking_3dcpp/scene.hpp>
#include <H5Cpp.h>
#include <benchmarking_3dcpp/viz/benchmarking_viz.hpp>
#include <benchmarking_3dcpp/robot_model/robot_model.hpp>

struct Task
{
    int task_id;
    RobotConfig robot;
    SceneConfig scene;
    std::shared_ptr<GeometryData> p_surface;
    AlgorithmConfig algorithm;

    // Detailed Coverage situation for inspection. 
    // Each row: the index of the surface point
    // the i-th row: the indices of robot waypoints that can cover this point
    std::vector<std::vector<int> > coverage_indices;

    CoverageResult result;
};

class CoverageEvaluator 
{
public:
    CoverageEvaluator(bool use_cuda = true, double point_density=1000);
    ~CoverageEvaluator();
    
    int countContinuousCoverage(const std::vector<int>& coverage_indices);

    void eval(int current_test_id, 
        const std::shared_ptr<RobotModel>& p_robot_model,
        const std::vector<SurfacePoint>& surface_points);

    void registerATest(int index, const std::string& robot_name, const std::string& scene_name, 
        const std::string& algorithm_name, const YAML::Node& config, 
        const std::unordered_map<std::string, std::shared_ptr<Scene> >& scenes);

    int getCoverageRadius(int current_test_id)
    {
        return tasks_[current_test_id].robot.tool_radius;
    }

    const Task& getTask(int test_id)
    {
        return tasks_[test_id];
    }

    Task& getTaskNonConst(int test_id)
    {
        return tasks_[test_id];
    }

    void setSolution(int test_id, std::shared_ptr<CoverageResult> solution)
    {
        tasks_[test_id].result = *solution;
    }

    inline int getTaskNum() const
    {
        return tasks_.size();
    }

    std::vector<std::vector<int>> evaluateCoverage(
        std::shared_ptr<RobotModel> robot,
        const std::vector<SurfacePoint>& surface_points,
        const std::vector<RobotWaypoint>& path) {
#ifdef USE_CUDA
        if (use_cuda_) {
            return calculateCoverageCUDA(surface_points, path, robot);
        } else {
            return calculateCoverageCPU(robot, surface_points, path);
        }
#else
        return calculateCoverageCPU(robot, surface_points, path);
#endif
        }

private:

    std::vector<Task> tasks_;

    bool use_cuda_;
    std::unique_ptr<SurfaceSampler> sampler_;
    
    std::vector<std::vector<int> > calculateCoverageCPU(
        const std::shared_ptr<RobotModel>& p_robot_model,
        const std::vector<SurfacePoint>& surface_points,
        const std::vector<RobotWaypoint>& path);
        
#ifdef USE_CUDA 
    std::vector<std::vector<int> > calculateCoverageCUDA(
        const std::vector<SurfacePoint>& surface_points,
        const std::vector<RobotWaypoint>& path,
        std::shared_ptr<RobotModel> the_tool);
#endif
};

ToolType getToolTypeFromString(const std::string& tool_name);

