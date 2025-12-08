#include <benchmarking_3dcpp/robot_model/line_lidar.hpp>

#include <string>

LineLidar::LineLidar(std::string robot_name, 
    double max_distance, double epsilon, 
    unsigned int beam_num, 
    std::vector<Eigen::Vector3d>& given_beam_vectors
):
    RobotModel(robot_name), beam_num_(beam_num), max_distance_(max_distance), 
    epsilon_(epsilon)
{
    // We pre-compute a set of beam orientations
    beam_directions_.resize(beam_num_, std::vector<float>(3, 0.0));

    for(size_t i = 0; i < beam_num_; i++)
    {
        for(size_t j = 0; j < 3; j++)
        {
            // We change to float
            beam_directions_[i][j] = given_beam_vectors[i][j];
        }
    }

}


bool LineLidar::isPointCovered(const SurfacePoint& point, const RobotWaypoint& waypoint) const
{
    // Based on the pre-calculated beam directions, 
    // we can check if a surface point is close enough to the beam
    // If so, it is treated as covered

    // 计算从传感器位置到目标点的向量
    // The direction from the sensor (robot waypoint) to the point of interest
    Eigen::Vector3d vec_to_point = point.position - waypoint.position;

    // 遍历所有预计算的光束方向
    // Loop through all pre-computed beam directions
    for (const auto& local_dir_vec : beam_directions_) 
    {
        // 将局部光束方向转换到世界坐标系
        // Convert the beam to the world frame (waypoint.orientation is the quaternion of the sensor in the world frame)
        Eigen::Vector3d beam_direction_world = waypoint.orientation * Eigen::Vector3d(local_dir_vec[0], local_dir_vec[1], local_dir_vec[2]);

        // 计算目标点在当前光束方向上的投影长度
        // 这相当于沿着光束方向走多远能离目标点“最近”
        // Compute the projection of the point of interest onto the beam
        double projection_length = vec_to_point.dot(beam_direction_world);

        // 检查投影点是否在光束的有效范围内
        // projection_length < 0: 点在传感器的“后方”，光束无法照到
        // projection_length > max_distance_: 点太远，超出了传感器量程
        if (projection_length < 0 || projection_length > max_distance_) 
        {
            // The point cannot be behind the sensor or too far away
            continue;
        }

        // 计算光束轴线上离目标点最近的点
        // Compute that closest point on the beam
        Eigen::Vector3d closest_point_on_beam = waypoint.position + projection_length * beam_direction_world;

        double distance_to_beam_axis = (point.position - closest_point_on_beam).norm();

        // epsilon_ 可以看作是光束的“厚度”或允许的测量误差
        // epsilon: the "thickness" of the beam, for the allowed measurement error
        if (distance_to_beam_axis <= epsilon_) {
            return true;
        }
    }

    return false;
}