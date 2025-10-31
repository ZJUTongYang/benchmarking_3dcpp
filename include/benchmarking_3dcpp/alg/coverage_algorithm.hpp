#pragma once
#include <Eigen/Dense>
#include <benchmarking_3dcpp/input_types.hpp>
#include <benchmarking_3dcpp/types.hpp>

class CoverageAlgorithm
{
public: 

    // virtual ~CoverageAlgorithm() = default;

    virtual std::string getName() const = 0;

    virtual std::string getDescription() const = 0;

    virtual std::vector<GeometryType> getSupportedInputTypes() const = 0;

    virtual CoverageResult execute(std::shared_ptr<GeometryData> input) = 0;

    virtual bool isValidInput(std::shared_ptr<GeometryData> input) const
    {
        // std::cout << "test 1.1" << std::endl;
        auto support_types = getSupportedInputTypes();
        // std::cout << "test 1.2" << std::endl;
        auto actual_type = input->getType();
        // std::cout << "test 1.3" << std::endl;
        return std::find(support_types.begin(), support_types.end(), actual_type) != support_types.end();
    }

public:
    // virtual std::vector<Eigen::Matrix4d> getResultPath() = 0;
    // virtual bool isCovered() = 0;
};
