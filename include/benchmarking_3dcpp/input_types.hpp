#pragma once

#include <memory>
#include <filesystem>
#include <functional>
#include <optional>
#include <open3d/Open3D.h>

namespace open3d{
    namespace geometry{
        class TriangleMesh;
        class PointCloud;
    }
}

enum class GeometryType{
    TriangleMesh,
    PointCloud
};

class GeometryData
{
public: 
    virtual ~GeometryData() = default;
    virtual GeometryType getType() const = 0;
    virtual bool isValid() const = 0;
};

class PointCloudData: public GeometryData
{
public: 
    explicit PointCloudData(std::shared_ptr<open3d::geometry::PointCloud> cloud):
        point_cloud_(std::move(cloud)){}

    GeometryType getType() const override {return GeometryType::PointCloud;}

    bool isValid() const override { return point_cloud_ != nullptr && !point_cloud_->points_.empty(); }

private: 
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud_;
};

class TriangleMeshData: public GeometryData
{
public: 
    explicit TriangleMeshData(std::shared_ptr<open3d::geometry::TriangleMesh> mesh):
        mesh_(std::move(mesh)){}

    GeometryType getType() const override {return GeometryType::TriangleMesh;}

    bool isValid() const override { return mesh_ != nullptr && !mesh_->vertices_.empty(); }

private:
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
};

class GeometryLoader
{
public: 
    using LoaderFunction = std::function<std::unique_ptr<GeometryData>(const std::string&)>;

    GeometryLoader()
    {
        initLoaders();
    }

    std::unique_ptr<GeometryData> load(const std::string& filename)
    {
        std::string extension = getFileExtension(filename);

        auto it = loaders_.find(extension);
        if(it != loaders_.end())
        {
            return it->second(filename);
        }

        open3d::utility::LogError("Unsupported file format: {}", filename);
        return nullptr;
    }

    std::unique_ptr<GeometryData> loadGeometry(const std::string& filename)
    {
        std::string extension = getFileExtension(filename);

        auto it = loaders_.find(extension);
        if(it != loaders_.end())
        {
            auto geometry = it->second(filename);
            if(geometry && geometry->isValid())
            {
                open3d::utility::LogInfo("Loaded geometry: {} ({})", 
                    filename, geometry->getType());
            }
        }
    }


    void registerLoader(const std::string& extension, LoaderFunction loader)
    {
        loaders_[extension] = std::move(loader);
    }

private: 
    std::unordered_map<std::string, LoaderFunction> loaders_;

    std::string getFileExtension(const std::string& filename)
    {
        size_t dotPos = filename.find_last_of('.');

        if(dotPos == std::string::npos)
        {
            return "";
        }

        std::string ext = filename.substr(dotPos+1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        return ext;
    }

    void initLoaders()
    {

        registerLoader("pcd", [this](const std::string& filename)->std::unique_ptr<GeometryData>{
            auto result = this->loadPCDFile(filename);
            if(result.has_value())
            {
                return std::make_unique<PointCloudData>(result.value());
            }
            return nullptr;
        });

        registerLoader("stl", [this](const std::string& filename)->std::unique_ptr<GeometryData>{
            auto result = this->loadSTLFile(filename);
            if(result.has_value())
            {
                return std::make_unique<TriangleMeshData>(result.value());
            }
            return nullptr;            
        });
    }




    std::optional<std::shared_ptr<open3d::geometry::TriangleMesh> >
    loadSTLFile(const std::string& filename) 
    {
        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        try{
            if(open3d::io::ReadTriangleMesh(filename, *mesh))
            {
                if(mesh->triangles_.empty())
                {
                    return std::nullopt;
                }

                mesh->RemoveDuplicatedVertices();
                mesh->RemoveDuplicatedTriangles();
                mesh->RemoveDegenerateTriangles();
                mesh->RemoveUnreferencedVertices();

                if(!mesh->HasVertexNormals())
                {
                    mesh->ComputeVertexNormals();
                }

                open3d::utility::LogInfo("Loaded mesh: {} ({} vertices, {} triangles)", 
                    filename, mesh->vertices_.size(), mesh->triangles_.size());

                return mesh;
            }
            else
            {
                return std::nullopt;
            }
        } catch(const std::exception& e){
            return std::nullopt;
        } // try

    }

    std::optional<std::shared_ptr<open3d::geometry::PointCloud> >
    loadPCDFile(const std::string& filename) 
    {
        auto pointcloud = std::make_shared<open3d::geometry::PointCloud>();

        try{
            if(open3d::io::ReadPointCloud(filename, *pointcloud))
            {
                if (pointcloud->points_.empty()) 
                {
                    return std::nullopt;
                }

                pointcloud->RemoveNonFinitePoints();
                pointcloud->RemoveDuplicatedPoints();

                if(!pointcloud->HasNormals() && !pointcloud->points_.empty())
                {
                    pointcloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
                }

                open3d::utility::LogInfo("Loaded point cloud: {} ({} points)", 
                    filename, pointcloud->points_.size());

                return pointcloud;
            }
            else
            {
                return std::nullopt;
            }
        } catch(const std::exception& e){
            return std::nullopt;
        } // try
    }


};