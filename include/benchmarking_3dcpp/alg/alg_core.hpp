#pragma once
#include <Eigen/Dense>

class CppAlgCore
{
    virtual std::vector<Eigen::Matrix4d> getResultPath() = 0;
    virtual bool isCovered() = 0;
};
