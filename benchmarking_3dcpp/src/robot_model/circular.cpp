#include <benchmarking_3dcpp/robot_model/circular.hpp>


bool Circular::isPointCovered(const SurfacePoint& point, const RobotWaypoint& waypoint) const
{
    // 1. 获取工具在世界坐标系下的主轴方向向量
    Eigen::Vector3d tool_direction = waypoint.orientation * -Eigen::Vector3d::UnitZ();
    
    // 2. 计算从工具中心到曲面点的向量
    Eigen::Vector3d vec_to_point = point.position - waypoint.position;

    // 3. 计算曲面点在工具轴线上的投影长度
    // 这相当于从工具中心出发，沿着工具方向需要走多远才能离曲面点“最近”
    double projection_length = vec_to_point.dot(tool_direction);
    // 如果距离的绝对值大于最大侵入距离 D_，则无法覆盖

    // 4. 检查投影点是否在工具的有效长度范围内 [0, D_]
    // projection_length < 0: 点在工具的“后方”（即+Z方向）
    // projection_length > D_: 点在工具“够不着”的地方
    if (projection_length < 0 || projection_length > D_) {
        return false;
    }

    // 5. 计算工具轴线上离曲面点最近的点
    Eigen::Vector3d closest_point_on_axis = waypoint.position + projection_length * tool_direction;

    // 6. 计算曲面点到工具轴线的最短距离（即径向距离）
    double radial_distance = (point.position - closest_point_on_axis).norm();

    // 7. 检查径向距离是否在覆盖半径 R_ 内
    if (radial_distance > R_) {
        return false;
    }

    // 如果所有条件都满足，则点被覆盖
    return true;
}