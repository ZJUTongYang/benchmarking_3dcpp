#include <benchmarking_3dcpp/robot_model/line_lidar.hpp>

#include <string>


// Helper function implementation
Eigen::Vector3d LineLidar::slerp(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t) const 
{
    // Ensure vectors are normalized
    Eigen::Vector3d v0 = start.normalized();
    Eigen::Vector3d v1 = end.normalized();

    // Compute the cosine of the angle between the vectors.
    // Clamp to [-1, 1] to handle floating-point inaccuracies.
    double dot = v0.dot(v1);
    dot = std::max(-1.0, std::min(1.0, dot));

    // Compute the angle (omega) between the vectors.
    double omega = std::acos(dot);

    // If the vectors are very close, use linear interpolation to avoid division by zero.
    // This is the case when sin(omega) is close to 0.
    if (omega < epsilon_) 
    {
        return (v0 * (1.0 - t) + v1 * t).normalized();
    }

    // Compute the interpolation
    double sin_omega = std::sin(omega);
    double term1 = std::sin((1.0 - t) * omega) / sin_omega;
    double term2 = std::sin(t * omega) / sin_omega;

    return (v0 * term1 + v1 * term2).normalized();
}

LineLidar::LineLidar(std::string robot_name, 
    double max_distance, double epsilon, 
    unsigned int beam_num, 
    std::vector<Eigen::Vector3d>& given_beam_vectors
):
    RobotModel(robot_name), beam_num_(beam_num), max_distance_(max_distance), 
    epsilon_(epsilon)
{
    if(beam_num_ < 1)
    {
        throw std::runtime_error("The number of beams must be at least 1");
    }

    // We normalize the beam vectors
    for(auto& vec : given_beam_vectors)
    {
        vec.normalize();
    }

    // We pre-compute a set of beam orientations
    beam_directions_.resize(beam_num_, std::vector<float>(3, 0.0));

    if(beam_num_ == given_beam_vectors.size())
    {
        for(size_t i = 0; i < beam_num_; i++)
        {
            for(size_t j = 0; j < 3; j++)
            {
                // We change to float
                beam_directions_[i][j] = given_beam_vectors[i][j];
            }
        }
    }
    else
    {
        
        // We do this by summing the angles between consecutive given vectors.
        std::vector<double> segment_angles;
        double total_angle = 0.0;
        for(size_t i = 0; i < given_beam_vectors.size() - 1; ++i)
        {
            const auto& prev = given_beam_vectors[i];
            const auto& next = given_beam_vectors[i+1];
            
            // Calculate the angle between the two unit vectors.
            // acos(dot_product) is the standard way.
            double dot_product = prev.dot(next);
            // Clamp for numerical stability
            dot_product = std::max(-1.0, std::min(1.0, dot_product));
            double angle = std::acos(dot_product);

            segment_angles.push_back(angle);
            total_angle += angle;
        }

        double target_segment_angle = total_angle / (beam_num_ - 1);


        double accumulated_angle = 0.0;
        size_t current_given_segment_idx = 0;
        
        // Generate beams from index 1 to beam_num_ - 1
        for (size_t i = 1; i < beam_num_; ++i) 
        {
            double target_angle_for_current_beam = i * target_segment_angle;

            // Find which original segment the target angle falls into.
            // We advance `current_given_segment_idx` until the accumulated angle
            // of the original segments surpasses our target angle.
            while (current_given_segment_idx < segment_angles.size() &&
                   accumulated_angle + segment_angles[current_given_segment_idx] < target_angle_for_current_beam) 
            {
                accumulated_angle += segment_angles[current_given_segment_idx];
                current_given_segment_idx++;
            }

            // If we run out of segments (e.g., due to floating point inaccuracies),
            // the last beam should be the last given vector.
            if (current_given_segment_idx >= segment_angles.size()) 
            {
                 for(size_t j = 0; j < 3; ++j) {
                    beam_directions_[i][j] = given_beam_vectors.back()[j];
                }
                continue;
            }
            
            // Now, we know the target angle is within the current segment.
            const auto& start_vec = given_beam_vectors[current_given_segment_idx];
            const auto& end_vec = given_beam_vectors[current_given_segment_idx + 1];
            
            // Calculate the interpolation parameter 't' for SLERP.
            // 't' represents the fraction of the way through the *current* segment.
            double angle_into_segment = target_angle_for_current_beam - accumulated_angle;
            double t = angle_into_segment / segment_angles[current_given_segment_idx];

            // Perform SLERP to get the new vector.
            Eigen::Vector3d new_vec = slerp(start_vec, end_vec, t);

            // Store the result, converting to float.
            for(size_t j = 0; j < 3; ++j) 
            {
                beam_directions_[i][j] = new_vec[j];
            }
        }
        
        // As a final safeguard, ensure the last beam is exactly the last given vector.
        // This handles any potential cumulative floating-point drift.
        for(size_t j = 0; j < 3; ++j) 
        {
            beam_directions_[beam_num_ - 1][j] = given_beam_vectors.back()[j];
        }

    } // else


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