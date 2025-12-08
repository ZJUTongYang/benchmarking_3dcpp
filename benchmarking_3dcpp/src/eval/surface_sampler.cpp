#include <benchmarking_3dcpp/eval/surface_sampler.hpp>
#include <open3d/geometry/TriangleMesh.h>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <benchmarking_3dcpp/input_types.hpp>

SurfaceSampler::SurfaceSampler(double point_density) 
    : point_density_(point_density), rng_(std::random_device{}()) 
{

}

std::shared_ptr<std::vector<SurfacePoint> > SurfaceSampler::prepareSurfacePoints(std::shared_ptr<GeometryData> p_scene_object)
{
    std::vector<SurfacePoint> surface_points;
    if(p_scene_object->getType() == GeometryType::TriangleMesh)
    {
        // Sample surface points
        const auto* mesh_data = static_cast<const TriangleMeshData*>(p_scene_object.get());

        surface_points = this->sampleUniformly(*(mesh_data->getData()));

        return std::make_shared<std::vector<SurfacePoint> >(surface_points);
    }
    else if(p_scene_object->getType() == GeometryType::PointCloud)
    {
        throw std::runtime_error("NotImplemented: line 54");
        // We directly use the point cloud as the surface points
        // std::cout << "We haven't implemented this branch" << std::endl;
        // const auto* cloud_data = static_cast<const PointCloudData*>(surface.get());

        // if(!cloud_data->isValid())
        // {
        //     tasks_[current_test_id].result = {};
        //     return;
        // }

        // surface_points = sampler_->sampleUniformly(*(cloud_data->getData()));

        //     tasks_[current_test_id].result = {};
        //     return;
    }
    else
    {
        throw std::runtime_error("NotImplemented: surface sampler line 54");
    }
}

std::vector<SurfacePoint> SurfaceSampler::sampleUniformly(
    const open3d::geometry::TriangleMesh& mesh) {
    
    auto triangle_areas = computeTriangleAreas(mesh);
    double total_area = std::accumulate(triangle_areas.begin(), 
                                       triangle_areas.end(), 0.0);
    return rejectionSamplingCPU(mesh, triangle_areas, total_area);
}

std::vector<SurfacePoint> SurfaceSampler::sampleUniformly(
    const open3d::geometry::PointCloud& pointcloud)
{
    (void)pointcloud;

    throw std::runtime_error("NotImplemented: SurfaceSampler::sampleUniformly");

    std::cout << "We have not implemented SurfaceSampler::sampleUniformly(const open3d::geometry::PointCloud & pointcloud)" << std::endl;
    return std::vector<SurfacePoint>();
}

std::vector<double> SurfaceSampler::computeTriangleAreas(
    const open3d::geometry::TriangleMesh& mesh) const {
    
    const auto& vertices = mesh.vertices_;
    const auto& triangles = mesh.triangles_;
    std::vector<double> areas(triangles.size());
    
    #pragma omp parallel for
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        const auto& v0 = vertices[tri[0]];
        const auto& v1 = vertices[tri[1]];
        const auto& v2 = vertices[tri[2]];
        
        Eigen::Vector3d edge1 = v1 - v0;
        Eigen::Vector3d edge2 = v2 - v0;
        areas[i] = 0.5 * edge1.cross(edge2).norm();
    }
    
    return areas;
}

std::vector<SurfacePoint> SurfaceSampler::rejectionSamplingCPU(
    const open3d::geometry::TriangleMesh& mesh,
    const std::vector<double>& triangle_areas,
    double total_area) 
{
    rng_.seed(1);
    const size_t target_points = static_cast<size_t>(total_area * point_density_);
    std::vector<SurfacePoint> samples;
    samples.reserve(target_points);
    
    const auto& vertices = mesh.vertices_;
    const auto& triangles = mesh.triangles_;
    const auto& vertex_normals = mesh.vertex_normals_;
    
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    
    // Create discrete distribution for triangle selection based on area
    std::vector<double> cumulative_areas(triangle_areas.size());
    std::partial_sum(triangle_areas.begin(), triangle_areas.end(), 
                     cumulative_areas.begin());

    for (size_t i = 0; i < target_points; ++i) {
        // Select triangle proportional to area
        double r = dist(rng_) * total_area;
        auto it = std::lower_bound(cumulative_areas.begin(), 
                                  cumulative_areas.end(), r);
        size_t tri_idx = std::distance(cumulative_areas.begin(), it);
        
        const auto& tri = triangles[tri_idx];
        const auto& v0 = vertices[tri[0]];
        const auto& v1 = vertices[tri[1]];
        const auto& v2 = vertices[tri[2]];
        
        // Generate random point in triangle
        Eigen::Vector3d point = randomPointInTriangle(v0, v1, v2);
        
        // Interpolate normal
        Eigen::Vector3d normal;
        if (!vertex_normals.empty()) {
            double u = dist(rng_), v = dist(rng_);
            if (u + v > 1.0) { u = 1.0 - u; v = 1.0 - v; }
            double w = 1.0 - u - v;
            normal = u * vertex_normals[tri[0]] + 
                    v * vertex_normals[tri[1]] + 
                    w * vertex_normals[tri[2]];
            normal.normalize();
        } else {
            normal = (v1 - v0).cross(v2 - v0).normalized();
        }
        
        samples.push_back({point, normal, static_cast<int>(tri_idx), false});
    }

    return samples;
}

Eigen::Vector3d SurfaceSampler::randomPointInTriangle(
    const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double u = dist(rng_), v = dist(rng_);
    
    if (u + v > 1.0) {
        u = 1.0 - u;
        v = 1.0 - v;
    }
    double w = 1.0 - u - v;
    
    return u * v0 + v * v1 + w * v2;
}