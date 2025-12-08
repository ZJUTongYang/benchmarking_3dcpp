#include <benchmarking_3dcpp/utils/h5_helper.hpp>
#include <H5Cpp.h>
#include <string>
#include <iostream>

void loadMetaData(H5::H5File& file, std::string& robot_name, 
    std::string& scene_name, std::string& alg_name)
{
    H5::StrType str_type(H5::PredType::C_S1, 256);
    robot_name.resize(256);
    scene_name.resize(256);
    alg_name.resize(256);

    H5::Attribute robot_attr = file.openAttribute("robot_name");
    robot_attr.read(str_type, robot_name.data());
    robot_name = robot_name.c_str(); // Trim null characters

    H5::Attribute surface_attr = file.openAttribute("surface_name");
    surface_attr.read(str_type, scene_name.data());
    scene_name = scene_name.c_str();

    H5::Attribute alg_attr = file.openAttribute("algorithm_name");
    alg_attr.read(str_type, alg_name.data());
    alg_name = alg_name.c_str();
}

void saveMetaData(H5::H5File& file, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name)
{
    // We need to save metadata
    H5::DataSpace attr_dataspace(H5S_SCALAR);
    H5::StrType str_type(H5::PredType::C_S1, 256);
    
    H5::Attribute robot_attr = file.createAttribute("robot_name", str_type, attr_dataspace);
        robot_attr.write(str_type, robot_name);
        
    H5::Attribute surface_attr = file.createAttribute("surface_name", str_type, attr_dataspace);
        surface_attr.write(str_type, scene_name);

    H5::Attribute alg_attr = file.createAttribute("algorithm_name", str_type, attr_dataspace);
        alg_attr.write(str_type, alg_name);
}

void loadRobotPath(H5::H5File& file, std::vector<RobotWaypoint>& robot_path)
{
    H5::DataSet dataset = file.openDataSet("/robot_path");
    H5::DataSpace dataspace = dataset.getSpace();
    
    hsize_t dims[2];
    dataspace.getSimpleExtentDims(dims, NULL);
    
    robot_path.resize(dims[0]);
    std::vector<double> data(dims[0] * 7);
    dataset.read(data.data(), H5::PredType::NATIVE_DOUBLE);

    for (size_t i = 0; i < dims[0]; ++i) {
        robot_path[i].position = Eigen::Vector3d(data[i*7+0], data[i*7+1], data[i*7+2]);
        robot_path[i].orientation = Eigen::Quaterniond(data[i*7+6], data[i*7+3], data[i*7+4], data[i*7+5]);
    }
}

void saveRobotPath(H5::H5File& file, const std::vector<RobotWaypoint>& robot_path)
{
    hsize_t dims[2] = {robot_path.size(), 7};

    H5::DataSpace dataspace(2, dims);

    H5::FloatType datatype(H5::PredType::NATIVE_DOUBLE);

    H5::DataSet dataset = file.createDataSet("/robot_path", datatype, dataspace);

    std::vector<double> data;
    data.resize(robot_path.size() * 7);

    for (size_t i = 0; i < robot_path.size(); i++)
    {
        data[i * 7 + 0] = robot_path[i].position.x();
        data[i * 7 + 1] = robot_path[i].position.y();
        data[i * 7 + 2] = robot_path[i].position.z();
        data[i * 7 + 3] = robot_path[i].orientation.x();
        data[i * 7 + 4] = robot_path[i].orientation.y();
        data[i * 7 + 5] = robot_path[i].orientation.z();
        data[i * 7 + 6] = robot_path[i].orientation.w();
    }
    dataset.write(data.data(), datatype);
}

void loadSurfacePointData(H5::H5File& file, std::vector<SurfacePoint>& surface_points)
{
    H5::DataSet dataset = file.openDataSet("/surface_points");
    H5::DataSpace dataspace = dataset.getSpace();
    
    hsize_t dims[2];
    dataspace.getSimpleExtentDims(dims, NULL);
    
    surface_points.resize(dims[0]);
    std::vector<double> data(dims[0] * 3);
    dataset.read(data.data(), H5::PredType::NATIVE_DOUBLE);

    for (size_t i = 0; i < dims[0]; ++i) {
        surface_points[i].position = Eigen::Vector3d(data[i*3+0], data[i*3+1], data[i*3+2]);
    }
}

void saveSurfacePointData(H5::H5File& file, const std::vector<SurfacePoint>& surface_points)
{
    hsize_t dims[2] = {surface_points.size(), 3};

    H5::DataSpace dataspace(2, dims);

    H5::FloatType datatype(H5::PredType::NATIVE_DOUBLE);

    H5::DataSet dataset = file.createDataSet("/surface_points", datatype, dataspace);

    std::vector<double> data;
    data.resize(surface_points.size() * 3);

    for (size_t i = 0; i < surface_points.size(); i++)
    {
        data[i * 3 + 0] = surface_points[i].position.x();
        data[i * 3 + 1] = surface_points[i].position.y();
        data[i * 3 + 2] = surface_points[i].position.z();
    }

    dataset.write(data.data(), datatype);
}

void saveCoverageIndices(H5::H5File& file, const std::vector<std::vector<int>>& coverage_indices)
{
    if (coverage_indices.empty()) 
    {
        return;
    }

    H5::Group coverage_group = file.createGroup("/coverage_indices");   

    int num_surface_points = coverage_indices.size();
    int max_covered_num = 0;
    for(auto iter = coverage_indices.begin(); iter != coverage_indices.end(); iter++)
    {
        max_covered_num = std::max(max_covered_num, (int)iter->size());
    }

    hsize_t dims[2] = {num_surface_points, max_covered_num};
    H5::DataSpace dataspace(2, dims);

    H5::DataSet dataset = coverage_group.createDataSet("coverage_indices", 
                                                       H5::PredType::NATIVE_INT, 
                                                       dataspace
                                                    );

    std::vector<int> flat_data(coverage_indices.size() * max_covered_num, -1);

    for(size_t i = 0; i < coverage_indices.size(); i++)
    {
        for(size_t j = 0; j < coverage_indices[i].size(); j++)
        {
            flat_data[i * max_covered_num + j] = coverage_indices[i][j];
        }
    }

    dataset.write(flat_data.data(), H5::PredType::NATIVE_INT);

}

void saveCoverageResult(H5::H5File& file, const CoverageResult& the_result)
{
    H5::Group coverage_group = file.createGroup("/coverage_result");

    H5::DataSpace scalar_dataspace(H5S_SCALAR);

    H5::Attribute coverage_attr = coverage_group.createAttribute("coverage_ratio", 
                                                                H5::PredType::NATIVE_DOUBLE, 
                                                                scalar_dataspace);
    coverage_attr.write(H5::PredType::NATIVE_DOUBLE, &the_result.coverage_ratio);


    H5::Attribute time_attr = coverage_group.createAttribute("computation_time", 
                                                            H5::PredType::NATIVE_DOUBLE, 
                                                            scalar_dataspace);
    time_attr.write(H5::PredType::NATIVE_DOUBLE, &the_result.computation_time);

    H5::Attribute total_attr = coverage_group.createAttribute("total_points", 
                                                                H5::PredType::NATIVE_HSIZE, 
                                                                scalar_dataspace);
    hsize_t total_points = the_result.total_points;
    total_attr.write(H5::PredType::NATIVE_HSIZE, &total_points);


    H5::Attribute covered_attr = coverage_group.createAttribute("covered_points", 
                                                                H5::PredType::NATIVE_HSIZE, 
                                                                scalar_dataspace);
    hsize_t covered_points = the_result.covered_points;
    covered_attr.write(H5::PredType::NATIVE_HSIZE, &covered_points);

    // save the number of times that each surface point is continuously covered
    if (!the_result.point_covered_num.empty()) 
    {
        hsize_t dims[1] = {the_result.point_covered_num.size()};
        H5::DataSpace dataspace(1, dims);
        
        H5::DataSet coverage_data = coverage_group.createDataSet("point_covered_num",
                                                                H5::PredType::NATIVE_INT,
                                                                dataspace);
        coverage_data.write(the_result.point_covered_num.data(),
                          H5::PredType::NATIVE_INT);
    }
}

void loadCoverageResult(H5::H5File& file, CoverageResult& the_result)
{
    H5::Group coverage_group = file.openGroup("/coverage_result");

    // 读取标量属性
    H5::Attribute coverage_attr = coverage_group.openAttribute("coverage_ratio");
    coverage_attr.read(H5::PredType::NATIVE_DOUBLE, &the_result.coverage_ratio);

    H5::Attribute time_attr = coverage_group.openAttribute("computation_time");
    time_attr.read(H5::PredType::NATIVE_DOUBLE, &the_result.computation_time);

    H5::Attribute total_attr = coverage_group.openAttribute("total_points");
    total_attr.read(H5::PredType::NATIVE_HSIZE, &the_result.total_points);

    H5::Attribute covered_attr = coverage_group.openAttribute("covered_points");
    covered_attr.read(H5::PredType::NATIVE_HSIZE, &the_result.covered_points);

    // 读取点覆盖次数数据集
    if (coverage_group.nameExists("point_covered_num")) {
        H5::DataSet coverage_data = coverage_group.openDataSet("point_covered_num");
        H5::DataSpace dataspace = coverage_data.getSpace();
        hsize_t dims[1];
        dataspace.getSimpleExtentDims(dims, NULL);
        
        the_result.point_covered_num.resize(dims[0]);
        coverage_data.read(the_result.point_covered_num.data(), H5::PredType::NATIVE_INT);
    }
}

bool loadFromHDF5(const std::string& filename, 
    std::string& robot_name, std::string& scene_name, std::string& alg_name,
    std::shared_ptr<std::vector<SurfacePoint>>& surface_points,
    CoverageResult& the_result)
{
    try
    {
        H5::H5File file(filename, H5F_ACC_RDONLY);
        loadMetaData(file, robot_name, scene_name, alg_name);
        loadRobotPath(file, the_result.robot_path);
        
        surface_points = std::make_shared<std::vector<SurfacePoint>>();
        loadSurfacePointData(file, *surface_points);

        loadCoverageResult(file, the_result);

        return true;
    }
    catch(const H5::FileIException& e)
    {
        std::cerr << "HDF5 File Error: " << e.getDetailMsg() << std::endl;
        return false;
    }
    catch(const std::exception& e)
    {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
        return false;
    }
}

bool saveToHDF5(const std::string& filename, const std::string& robot_name, 
    const std::string& scene_name, const std::string& alg_name, 
    const std::shared_ptr<std::vector<SurfacePoint> >& surface_points,
    const std::vector<std::vector<int> >& the_coverage_indices,
    const CoverageResult& the_result)
{
    try
    {
        H5::H5File file(filename, H5F_ACC_TRUNC);
        saveMetaData(file, robot_name, scene_name, alg_name);
        saveRobotPath(file, the_result.robot_path);
        saveSurfacePointData(file, *surface_points);
        saveCoverageIndices(file, the_coverage_indices);
        saveCoverageResult(file, the_result);
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}