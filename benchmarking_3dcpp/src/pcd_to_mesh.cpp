// #include <open3d/Open3D.h>

// using namespace open3d;

// std::shared_ptr<geometry::TriangleMesh> ConvertPCDToMeshBallPivoting(
//     const std::string& pcd_filename) {
    
//     // Load point cloud
//     auto cloud = io::ReadPointCloud(pcd_filename);
//     if (!cloud || cloud->points_.empty()) {
//         utility::LogError("Failed to load PCD file or file is empty");
//         return nullptr;
//     }
    
//     // Estimate normals (required for ball pivoting)
//     cloud->EstimateNormals();
    
//     // Ball pivoting algorithm
//     std::vector<double> radii = {0.05, 0.1, 0.2, 0.4};
//     auto mesh = geometry::TriangleMesh::CreateFromPointCloudBallPivoting(
//         *cloud, radii);
    
//     return mesh;
// }

// #include <open3d/Open3D.h>

// using namespace open3d;

// std::shared_ptr<geometry::TriangleMesh> ConvertPCDToMeshPoisson(
//     const std::string& pcd_filename) {
    
//     // Load point cloud
//     auto cloud = io::ReadPointCloud(pcd_filename);
//     if (!cloud || cloud->points_.empty()) {
//         utility::LogError("Failed to load PCD file or file is empty");
//         return nullptr;
//     }
    
//     // Estimate normals
//     cloud->EstimateNormals();
    
//     // Poisson surface reconstruction
//     auto mesh = std::get<0>(geometry::TriangleMesh::CreateFromPointCloudPoisson(*cloud));
    
//     return mesh;
// }

// #include <open3d/Open3D.h>

// using namespace open3d;

// std::shared_ptr<geometry::TriangleMesh> ConvertPCDToMeshAlphaShape(
//     const std::string& pcd_filename, double alpha = 0.03) {
    
//     // Load point cloud
//     auto cloud = io::ReadPointCloud(pcd_filename);
//     if (!cloud || cloud->points_.empty()) {
//         utility::LogError("Failed to load PCD file or file is empty");
//         return nullptr;
//     }
    
//     // Alpha shape reconstruction
//     auto mesh = geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*cloud, alpha);
    
//     return mesh;
// }