#pragma once
#include <Eigen/Dense>
#include <benchmarking_3dcpp/input_types.hpp>
#include <benchmarking_3dcpp/types.hpp>

class CoverageAlgorithm
{
public: 
    CoverageAlgorithm()
    {
        std::shared_ptr<CoverageResult> result = nullptr;
    }

    virtual std::string getName() const = 0;

    virtual std::string getDescription() const = 0;

    virtual std::vector<GeometryType> getSupportedInputTypes() const = 0;

    virtual void execute(std::shared_ptr<GeometryData> input) = 0;

    virtual bool isValidInput(std::shared_ptr<GeometryData> input) const
    {
        auto support_types = getSupportedInputTypes();
        auto actual_type = input->getType();
        return std::find(support_types.begin(), support_types.end(), actual_type) != support_types.end();
    }

    // For any algorithm, the coverage result is not get immediately. 
    // So they will save the solution later using this function 
    void setSolution(std::shared_ptr<CoverageResult> result)
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_ = result;
    }
    
    std::shared_ptr<CoverageResult> getSolution()
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        return result_;
    }

public:
    std::chrono::_V2::system_clock::time_point start_time_;
    std::chrono::_V2::system_clock::time_point end_time_;

    std::mutex result_mutex_;
    std::shared_ptr<CoverageResult> result_;

};
