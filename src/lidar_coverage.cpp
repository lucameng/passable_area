#include "lidar_coverage.hpp"
#include "elevation_map.hpp"
#include "dr_math.hpp"
#include "dr_utils.hpp"

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

LidarCoverage::LidarCoverage(const ros::NodeHandle& nh)
    : nh_(nh),
      ground_height_(-0.5f),
      bound_max_({4.f, 1.f}),
      bound_min_({-2.f, -1.f})
{
    // LidarCoverage constructor
}

void LidarCoverage::initialize()
{
    // imu_sub_ = nh_.subscribe("imu", 1000, &LidarCoverage::imuCallback, this);
    addLidar("lidar_front_up");
    addLidar("lidar_front_down");
    addLidar("lidar_rear_up");
    addLidar("lidar_rear_down");
}

// void LidarCoverage::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     Eigen::Quaternionf q(msg->orientation.w, msg->orientation.x, msg->orientation.y,
//                          msg->orientation.z);
//     q.normalize();

//     Eigen::Matrix3f R = q.toRotationMatrix();
//     ROS_INFO_STREAM("R: \n" << R);

//     T_g2b_.setIdentity();
//     T_g2b_.linear() = R;
//     T_g2b_.translation() = Eigen::Vector3f::Zero();
// }

void LidarCoverage::addLidar(const std::string& lidar_name)
{
    LidarParam param;
    param.name = lidar_name;

    std::vector<float> pos_vec;
    if (nh_.getParam(lidar_name + "/pos_body", pos_vec) && pos_vec.size() == 3)
    {
        param.pos_body = Eigen::Vector3f(pos_vec[0], pos_vec[1], pos_vec[2]);
    }
    else
    {
        ROS_ERROR("Failed to get position for %s", lidar_name.c_str());
    }

    std::vector<float> rpy_vec;
    if (nh_.getParam(lidar_name + "/rpy_body", rpy_vec) && rpy_vec.size() == 3)
    {
        param.rpy_body_deg = Eigen::Vector3f(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
        param.rpy_body_rad = Eigen::Vector3f(dr_math::degreeToRadian(rpy_vec[0]),
                                             dr_math::degreeToRadian(rpy_vec[1]),
                                             dr_math::degreeToRadian(rpy_vec[2]));
        param.R_mount = dr_utils::rotationFromYPRrad(param.rpy_body_rad);
        ROS_INFO_STREAM(lidar_name << ":\nrpy:\n" << param.rpy_body_rad 
                        << "\n\nrotation matrix:\n"<< param.R_mount << "\n");
    }
    else
    {
        ROS_ERROR("Failed to get euler angle for %s", lidar_name.c_str());
    }

    nh_.getParam(lidar_name + "/fov_up_deg", param.fov_up_deg);
    nh_.getParam(lidar_name + "/fov_down_deg", param.fov_down_deg);
    param.fov_up_rad = dr_math::degreeToRadian(param.fov_up_deg);
    param.fov_down_rad = dr_math::degreeToRadian(param.fov_down_deg);

    lidars_.push_back(param);
}

bool LidarCoverage::isCellCoveredByLidar(const Eigen::Vector3f& p_body,
                                         const LidarParam& lidar) const
{
    Eigen::Vector3f v_body = p_body - lidar.pos_body;
    float dist = v_body.norm();
    if (dist < lidar.min_range || dist > lidar.max_range) return false;

    Eigen::Vector3f v_lidar = lidar.R_mount.transpose() * v_body;
    // Eigen::Vector3f v_lidar = lidar.R_mount * v_body;

    // if (v_lidar.x() <= 0.0) return false;
    // ROS_INFO_STREAM("\nv_body:\n" << v_body << "\nv_lidar:\n"<< v_lidar << "\n");
    float horiz = std::hypot(v_lidar.x(), v_lidar.y());
    float theta = std::atan2(v_lidar.z(), horiz); // rad

    // ROS_INFO("horiz=%.3f theta=%.3f", horiz, dr_math::radianToDegree(theta));
    if (theta >= lidar.fov_down_rad && theta <= lidar.fov_up_rad) return true;
    return false;
}

void LidarCoverage::computeCoverage(grid_map::GridMap& map, const Eigen::Affine3f& T_g2b) const
{
    // ROS_INFO("Start computing lidar coverage...");
    map.get("coverability").setConstant(UNCOVERED);

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        const grid_map::Index idx = *it;
        grid_map::Position3 pos;
        map.getPosition3("elevation", idx, pos);

        // if (pos.x() > bound_max_.x() || pos.y() > bound_max_.y() ||
        //     pos.x() < bound_min_.x() || pos.y() < bound_min_.y())
        //     continue;

        if (dr_math::equal(pos.z(), DEAD_VALUE))
        {
            pos.z() = ground_height_;
        }

        Eigen::Vector3f p_grav = {static_cast<float>(pos.x()),
                                  static_cast<float>(pos.y()), 
                                  static_cast<float>(pos.z())};
        Eigen::Vector3f p_body = T_g2b * p_grav;
        
        // ROS_INFO("p_grav:(%.3f, %.3f, %.3f) p_body(%.3f, %.3f, %.3f)", 
        //         p_grav.x(), p_grav.y(), p_grav.z(), p_body.x(), p_body.y(), p_body.z());
        for (const auto& lidar : lidars_)
        {
            if (isCellCoveredByLidar(p_body, lidar))
            {
                map.at("coverability", idx) = COVERED;
                break;
            }
        }
    }

    // ROS_INFO("Computation done!");
}