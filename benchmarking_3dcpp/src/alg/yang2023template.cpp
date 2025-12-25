#include <benchmarking_3dcpp/alg/yang2023template.hpp>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <chrono>
#include <thread>

#include <benchmarking_3dcpp_interfaces/srv/get_nuc.hpp>

using namespace std::chrono_literals;

Yang2023Template::Yang2023Template(rclcpp::Node::SharedPtr node): 
    node_(node) 
{
    // Create client    
    std::string service_name = "get_"+getName()+"_benchmark";
    client_ = node_->create_client<benchmarking_3dcpp_interfaces::srv::GetNuc>(service_name);
    
    std::string msg = getName() + " Client initialized";
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
}

void Yang2023Template::execute(std::shared_ptr<GeometryData> input)
{
    std::cout << "We start executing " << getName() << std::endl;
    if(!isValidInput(input))
    {
        std::cout << "Input data to the algorithm is not suitable" << std::endl;
        return ;
    }

    start_time_ = std::chrono::high_resolution_clock::now();

    benchmarking_3dcpp_interfaces::srv::GetNuc::Request request;

    std::shared_ptr<TriangleMeshData> p_tri_mesh_data = std::dynamic_pointer_cast<TriangleMeshData>(input);
    std::shared_ptr<open3d::geometry::TriangleMesh> p_mesh = p_tri_mesh_data->getData();

    const auto& triangles = p_mesh->triangles_;
    request.mesh.triangles.resize(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i) {
        request.mesh.triangles[i].vertex_indices[0] = triangles[i][0];
        request.mesh.triangles[i].vertex_indices[1] = triangles[i][1];
        request.mesh.triangles[i].vertex_indices[2] = triangles[i][2];
    }

    // Convert vertices
    const auto& vertices = p_mesh->vertices_;
    request.mesh.vertices.resize(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        request.mesh.vertices[i].x = vertices[i].x();
        request.mesh.vertices[i].y = vertices[i].y();
        request.mesh.vertices[i].z = vertices[i].z();
    }
    
    auto request_ptr = std::make_shared<benchmarking_3dcpp_interfaces::srv::GetNuc::Request>(request);

    // We don't know when the service will be ready, so we have to set up a callback function
    auto result_future = client_->async_send_request(request_ptr, 
        std::bind(&Yang2023Template::resultCallback, this, std::placeholders::_1));

}

void Yang2023Template::resultCallback(rclcpp::Client<benchmarking_3dcpp_interfaces::srv::GetNuc>::SharedFuture future)
{
    std::cout << "the platform just gets the path callback. " << std::endl;
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time_);

    auto response = future.get();

    auto result = std::make_shared<CoverageResult>();
    for(auto iter = response->coverage.poses.begin(); iter != response->coverage.poses.end(); ++iter)
    {
        result->robot_path.emplace_back(RobotWaypoint(*iter));
    }

    result->computation_time = duration.count();
    setSolution(result);

    std::cout << "Finish executing " << getName() << " with computation time " << result_->computation_time << "s." << std::endl;
}

std::shared_ptr<open3d::geometry::TriangleMesh> Yang2023Template::loadMesh(const std::string& file_path) 
{
    auto mesh = open3d::io::CreateMeshFromFile(file_path);
    if (!mesh) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh from: %s", file_path.c_str());
        return nullptr;
    }
    
    // Ensure mesh has normals for proper processing
    if (!mesh->HasVertexNormals()) {
        mesh->ComputeVertexNormals();
    }
    
    RCLCPP_INFO(node_->get_logger(), "Mesh loaded: %zu vertices, %zu triangles", 
               mesh->vertices_.size(), mesh->triangles_.size());
    
    return mesh;
}
