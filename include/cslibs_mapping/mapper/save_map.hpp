#ifndef SAVE_MAP_HPP
#define SAVE_MAP_HPP

#include <cslibs_math_2d/linear/pose.hpp>
#include <nav_msgs/Path.h>

#include <cslibs_math_2d/serialization/transform.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>
#include <cslibs_math_ros/geometry_msgs/conversion_2d.hpp>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace cslibs_mapping {
template <typename T>
bool saveMap(
    const std::string            & occ_path_yaml,
    const std::string            & occ_path_pgm,
    const std::string            & occ_path_raw_pgm,
    const std::string            & poses_path_yaml,
    const nav_msgs::Path         & poses_path,
    const std::vector<T>         & occ_data,
    const std::size_t            & occ_height,
    const std::size_t            & occ_width,
    const cslibs_math_2d::Pose2d & occ_origin,
    const double                 & occ_resolution,
    const double                 & occ_free_threshold     = 0.196,
    const double                 & occ_occupied_threshold = 0.65)
{
    // map header
    {
        std::ofstream occ_out_yaml(occ_path_yaml);
        if (!occ_out_yaml.is_open()) {
            std::cout << "[SaveMap]: Could not open file '" << occ_path_yaml << "'." << std::endl;
            return false;
        }
        /// write occupancy map meta data
        YAML::Emitter occ_yaml(occ_out_yaml);
        occ_yaml << YAML::BeginMap;
        occ_yaml << YAML::Key   << "image";
        occ_yaml << YAML::Value << occ_path_pgm;
        occ_yaml << YAML::Key   << "resolution";
        occ_yaml << YAML::Value << occ_resolution;
        occ_yaml << YAML::Key   << "origin";
        occ_yaml << YAML::BeginSeq;
        occ_yaml << YAML::Flow;
        occ_yaml << occ_origin.tx();
        occ_yaml << occ_origin.ty();
        occ_yaml << occ_origin.yaw();
        occ_yaml << YAML::EndSeq;
        occ_yaml << YAML::Key   << "occupied_threshold";
        occ_yaml << YAML::Value << occ_occupied_threshold;
        occ_yaml << YAML::Key   << "free_threshold";
        occ_yaml << YAML::Value << occ_free_threshold;
        occ_yaml << YAML::Key   << "negate";
        occ_yaml << YAML::Value << 0;
        occ_yaml << YAML::EndMap;
        occ_out_yaml.close();
    }

    // pgm files
    {
        std::ofstream occ_out_pgm(occ_path_pgm);
        std::ofstream occ_out_raw_pgm(occ_path_raw_pgm);
        if (!occ_out_pgm.is_open()) {
            std::cout << "[SaveMap]: Could not open file '" << occ_path_pgm << "'." << std::endl;
            return false;
        }
        if (!occ_out_raw_pgm.is_open()) {
            std::cout << "[SaveMap]: Could not open file '" << occ_path_raw_pgm << "'." << std::endl;
            return false;
        }

        const double *occ_data_ptr = occ_data.data();
        const std::size_t occ_max_idx = occ_width - 1ul;

        /// write pgm headers
        occ_out_pgm << "P5 \n";
        occ_out_pgm << "# CREATOR: cslibs_mapping " << occ_resolution << "m/pix \n";
        occ_out_pgm << occ_width << " " << occ_height << "\n";
        occ_out_pgm << 255 << "\n";
        occ_out_raw_pgm << "P5 \n";
        occ_out_raw_pgm << "# CREATOR: cslibs_mapping " << occ_resolution << "m/pix \n";
        occ_out_raw_pgm << occ_width << " " << occ_height << "\n";
        occ_out_raw_pgm << 255 << "\n";

        auto convert_ros = [&occ_free_threshold, &occ_occupied_threshold](const double p) {
            if (cslibs_math::common::le(p, occ_free_threshold))
                return static_cast<uint8_t>(254);
            if (cslibs_math::common::le(p,occ_occupied_threshold))
                return static_cast<uint8_t>(0);
            return static_cast<uint8_t>(205);
        };
        auto convert_raw = [](const double p) {
            return static_cast<uint8_t>((1.0 - p) * 255);
        };
        for (std::size_t i = 0 ; i < occ_height ; ++i) {
            for (std::size_t j = 0 ; j < occ_max_idx; ++j) {
                occ_out_pgm << convert_ros(*occ_data_ptr);
                occ_out_raw_pgm << convert_raw(*occ_data_ptr);
                ++occ_data_ptr;
            }
            occ_out_pgm << convert_ros(*occ_data_ptr);
            occ_out_raw_pgm << convert_raw(*occ_data_ptr);
            ++occ_data_ptr;
        }

        occ_out_pgm.close();
        occ_out_raw_pgm.close();
    }

    // map data
    {
        const double occ_inv_resolution = 1.0 / occ_resolution;
        const cslibs_math_2d::Transform2d m_t_w = occ_origin.inverse();

        std::ofstream poses_out_yaml(poses_path_yaml);
        if (!poses_out_yaml.is_open()) {
            std::cout << "[SaveMap]: Could not open file '" << poses_path_yaml << "'." << std::endl;
            return false;
        }
        YAML::Emitter poses_yaml(poses_out_yaml);
        poses_yaml << YAML::BeginSeq;
        for (const auto &p_w : poses_path.poses) {
            const cslibs_math_2d::Transform2d p_m = m_t_w * cslibs_math_ros::geometry_msgs::conversion_2d::from(p_w.pose);
            poses_yaml << YAML::BeginSeq << YAML::Flow <<  p_m.tx() * occ_inv_resolution << p_m.ty() * occ_inv_resolution << YAML::EndSeq;
        }
        poses_yaml << YAML::EndSeq;
        poses_out_yaml.close();
    }

    return true;
}
}

#endif // SAVE_MAP_HPP
