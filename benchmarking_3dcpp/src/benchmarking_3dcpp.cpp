#include <benchmarking_3dcpp/benchmarking_3dcpp.hpp>
#include <open3d/Open3D.h>
#include <filesystem>
#include <string>
#include <memory>
#include <iostream>
#include <benchmarking_3dcpp/eval/coverage_evaluator.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <benchmarking_3dcpp/alg/coverage_algorithm.hpp>
#include <benchmarking_3dcpp/alg/yang2023template.hpp>
#include <benchmarking_3dcpp/robot_model/line_lidar.hpp>
#include <benchmarking_3dcpp/robot_model/circular.hpp>
#include <benchmarking_3dcpp/utils/h5_helper.hpp>

Benchmarking3DCPP::Benchmarking3DCPP(): 
    Node("benchmarking_3dcpp_node")
{
    initialized_ = false;
    geometryLoader_ = std::make_unique<GeometryLoader>();

    this->declare_parameter("config_filename", "NOT_SET");
    config_filename_ = this->get_parameter("config_filename").as_string();
    if(config_filename_ == "NOT_SET")
    {
        RCLCPP_ERROR(this->get_logger(), "Config filename not set");
        return;
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Benchmarking3DCPP::runSingleTest, this));
}

void Benchmarking3DCPP::initialize()
{
    YAML::Node config = YAML::LoadFile(config_filename_);

    test_is_running_= false; // A marker indicating that whether we should wait for the result of the "current task"
    curr_test_index_ = 0;

    double point_density = config["point_density"].as<double>(); // TODO: we need to find an automatical way for sampling
    surface_sampler_ = std::make_unique<SurfaceSampler>(point_density);

    // The benchmarking platform loads all scenes and only share pointers to the evaluator
    scenes_.clear();
    for(const auto& scene : config["scenes"])
    {
        std::string scene_name = scene["name"].as<std::string>();
        std::string scene_filename = scene["filename"].as<std::string>();
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmarking_3dcpp");
        std::filesystem::path scene_path = std::filesystem::path(package_share_directory) / "scene" / scene_filename;
        std::cout << "scene_path: " << scene_path.string() << std::endl;
        std::shared_ptr<GeometryData> p_scene_object = loadGeometryFile(scene_path.string());
        if(p_scene_object == nullptr)
        {
            std::cout << "YT: error, we cannot skip a scene file, because they are ordered" << std::endl;
            return ;
        }
        auto p_surface_points = surface_sampler_->prepareSurfacePoints(p_scene_object);
        auto p_scene = std::make_shared<Scene>(p_scene_object, p_surface_points);
        std::cout << "Number of surface points for scene [" << scene_name << "]: " << p_surface_points->size() << std::endl;
        scenes_[scene_name] = p_scene;
    }

    std::cout << "We create all robots." << std::endl;
    robots_.clear();
    for(const auto& robot : config["robots"])
    {
        std::string robot_name = robot["name"].as<std::string>();
        std::cout << "Robot Name: " << robot_name << std::endl;
        std::shared_ptr<RobotModel> p_robot;
        if(robot_name == "circular")
        {
            p_robot = std::make_shared<Circular>(robot_name, robot["radius"].as<double>(), robot["depth"].as<double>());
        }
        else if(robot_name == "line_lidar")
        {
            std::vector<std::pair<double, double> > temp;
            p_robot = std::make_shared<LineLidar>(robot_name, 
                robot["beam_num"].as<int>(), 
                robot["max_distance"].as<double>(),
                robot["pitch"].as<double>(),
                robot["yaw_range"].as<double>(),
                robot["epsilon"].as<double>());
        }
        else
        {
            std::cout << "Error, we do not support this robot: " << robot_name << std::endl;
            return ;
        }
        robots_[robot_name] = p_robot;
    }
    std::cout << "Finish creating all robots." << std::endl;

    std::cout << "We start creating algorithms." << std::endl;
    algorithms_.clear();
    for(const auto& algorithm : config["algorithms"])
    {
        std::string algorithm_name = algorithm["name"].as<std::string>();

        std::shared_ptr<CoverageAlgorithm> p_algorithm;
        if(algorithm_name == "Yang2023Template")
        {
            p_algorithm = std::make_shared<Yang2023Template>(this->shared_from_this());
        }
        else
        {
            std::cout << "Error, we do not support this algorithm: " << algorithm_name << std::endl;
            return ;
        }

        algorithms_[algorithm_name] = p_algorithm;
    }
    std::cout << "Finish creating all algorithms." << std::endl;

    // Initialize coverage calculator
    // double point_density = this->get_parameter("point_density").as_double();
    bool use_cuda = config["use_cuda"].as<bool>();
    benchmarker_ = std::make_unique<CoverageEvaluator>(use_cuda, point_density);

    int num_robots = config["robots"].size();
    int num_scenes = config["scenes"].size();
    int num_algorithms = config["algorithms"].size();
    
    scheduleAllTests(num_robots, num_scenes, num_algorithms, config);

    initialized_ = true;
}

void Benchmarking3DCPP::scheduleAllTests(int num_robots, int num_scenes, int num_algorithms, const YAML::Node& config)
{
    std::cout << "We start scheduling all tests. " << std::endl;
    for(int i = 0; i < num_robots; i++)
    {
        for(int j = 0; j < num_scenes; j++)
        {
            for(int k = 0; k < num_algorithms; k++)
            {
                int task_id = i * num_scenes * num_algorithms + j * num_algorithms + k;

                std::string robot_name = config["robots"][i]["name"].as<std::string>();
                std::string scene_name = config["scenes"][j]["name"].as<std::string>();
                std::string algorithm_name = config["algorithms"][k]["name"].as<std::string>();
                benchmarker_->registerATest(task_id, robot_name, scene_name, algorithm_name, config, scenes_);
            }
        }
    }
    std::cout << "We finish scheduling " << benchmarker_->getTaskNum() << " tests. " << std::endl;
}

void Benchmarking3DCPP::runSingleTest()
{
    if(!initialized_)
    {
        initialize();
        return ;
    }

    if(curr_test_index_ >= benchmarker_->getTaskNum())
    {
        // All tests have finished
        timer_->cancel();  
        return ;
    }

    // we perform the i-th test
    const RobotConfig& the_robot_config = benchmarker_->getTask(curr_test_index_).robot;
    const SceneConfig& the_scene_config = benchmarker_->getTask(curr_test_index_).scene;
    const std::shared_ptr<GeometryData>& p_the_surface = benchmarker_->getTask(curr_test_index_).p_surface;

    const AlgorithmConfig& the_algorithm_config = benchmarker_->getTask(curr_test_index_).algorithm;

    std::shared_ptr<CoverageAlgorithm> p_algorithm = algorithms_[the_algorithm_config.name];

    if(!test_is_running_)
    {
        p_algorithm->execute(p_the_surface);
        test_is_running_ = true;
        return;
    }
    else
    {
        // After we call "execute", the algorithm may run for a long time and terminated with a callback, so we are going to wait for the callback to finish

        if(p_algorithm->getSolution() == nullptr)
        {
            // We still need to wait a bit longer
            return ;
        }

        timer_->cancel();
        // We copy the result to the evaluator
        std::shared_ptr<CoverageResult> p_result = p_algorithm->getSolution();
        benchmarker_->setSolution(curr_test_index_, p_result);
        benchmarker_->eval(curr_test_index_, 
                            robots_[the_robot_config.name],
                            *(scenes_[the_scene_config.name]->surface_points_));
        saveEvalToFile(curr_test_index_);
        test_is_running_ = false;
        curr_test_index_++;
        timer_->reset();
    }
}

void Benchmarking3DCPP::saveEvalToFile(int task_id)
{
    const auto& the_task = benchmarker_->getTask(task_id);
    // We save the data to hdf5 dataset, which is small and fast to read
    std::string robot_name = the_task.robot.name;
    std::string scene_name = the_task.scene.name;
    std::string alg_name = the_task.algorithm.name;

    std::string filename = robot_name + "_" + scene_name + "_" + alg_name + ".h5";

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmarking_3dcpp");
    std::filesystem::path file_fullname = std::filesystem::path(package_share_directory) / "output" / filename;

    std::filesystem::path output_dir = file_fullname.parent_path();
    std::error_code ec;
    if (!std::filesystem::exists(output_dir, ec) && !std::filesystem::create_directories(output_dir, ec)) 
    {
        std::cerr << "Error: Could not create directory " << output_dir << ": " << ec.message() << std::endl;
        return;
    }

    bool success = saveToHDF5(file_fullname, the_task.robot.name, the_task.scene.name, 
            the_task.algorithm.name, 
            scenes_[the_task.scene.name]->surface_points_, 
            the_task.result);
    if(success)
    {
        std::cout << "Saved to " << file_fullname << std::endl;
    }
    else
    {
        std::cout << "Failed to save to " << file_fullname << std::endl;
    }
}
    