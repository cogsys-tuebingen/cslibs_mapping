#ifndef CSLIBS_MAPPING_SAVE_MAP_HPP
#define CSLIBS_MAPPING_SAVE_MAP_HPP

#include <cslibs_math_2d/linear/pose.hpp>
#include <nav_msgs/Path.h>

#include <cslibs_math_2d/serialization/transform.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>
#include <cslibs_math_ros/geometry_msgs/conversion_2d.hpp>
#include <cslibs_math/common/equal.hpp>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace cslibs_mapping {
namespace mapper {
template <typename Tp, typename T>
inline bool saveMap(
        const std::string                &path,
        const nav_msgs::Path::Ptr        &poses,
        const std::vector<T>             &occ_data,
        const std::size_t                &occ_height,
        const std::size_t                &occ_width,
        const cslibs_math_2d::Pose2<Tp>  &occ_origin,
        const Tp                         &occ_resolution,
        const T                          &occ_free_threshold     = 0.169,
        const T                          &occ_occupied_threshold = 0.65)
{
    const std::string occ_path_yaml        = (path / boost::filesystem::path("occ.map.yaml")).       string();
    const std::string occ_path_pgm_rel     = "occ.map.pgm";
    const std::string occ_path_pgm         = (path / boost::filesystem::path(occ_path_pgm_rel)).     string();
    const std::string occ_path_raw_yaml    = (path / boost::filesystem::path("occ.map.raw.yaml")).   string();
    const std::string occ_path_raw_pgm_rel = "occ.map.raw.pgm";
    const std::string occ_path_raw_pgm     = (path / boost::filesystem::path(occ_path_raw_pgm_rel)). string();
    const std::string poses_path_yaml      = (path / boost::filesystem::path("poses.yaml")).         string();

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
        occ_yaml << YAML::Value << occ_path_pgm_rel;
        occ_yaml << YAML::Key   << "resolution";
        occ_yaml << YAML::Value << occ_resolution;
        occ_yaml << YAML::Key   << "origin";
        occ_yaml << YAML::Flow;
        occ_yaml << YAML::BeginSeq << occ_origin.tx() << occ_origin.ty() << occ_origin.yaw() << YAML::EndSeq;
        occ_yaml << YAML::Key   << "occupied_thresh";
        occ_yaml << YAML::Value << occ_occupied_threshold;
        occ_yaml << YAML::Key   << "free_thresh";
        occ_yaml << YAML::Value << occ_free_threshold;
        occ_yaml << YAML::Key   << "negate";
        occ_yaml << YAML::Value << 0;
        occ_yaml << YAML::EndMap;
        occ_out_yaml.close();
    }

    // raw map header
    {
        std::ofstream occ_out_yaml_raw(occ_path_raw_yaml);
        if (!occ_out_yaml_raw.is_open()) {
            std::cout << "[SaveMap]: Could not open file '" << occ_path_raw_yaml << "'." << std::endl;
            return false;
        }
        /// write occupancy map meta data
        YAML::Emitter occ_yaml_raw(occ_out_yaml_raw);
        occ_yaml_raw << YAML::BeginMap;
        occ_yaml_raw << YAML::Key   << "image";
        occ_yaml_raw << YAML::Value << occ_path_raw_pgm_rel;
        occ_yaml_raw << YAML::Key   << "resolution";
        occ_yaml_raw << YAML::Value << occ_resolution;
        occ_yaml_raw << YAML::Key   << "origin";
        occ_yaml_raw << YAML::Flow;
        occ_yaml_raw << YAML::BeginSeq << occ_origin.tx() << occ_origin.ty() << occ_origin.yaw() << YAML::EndSeq;
        occ_yaml_raw << YAML::Key   << "occupied_thresh";
        occ_yaml_raw << YAML::Value << occ_occupied_threshold;
        occ_yaml_raw << YAML::Key   << "free_thresh";
        occ_yaml_raw << YAML::Value << occ_free_threshold;
        occ_yaml_raw << YAML::Key   << "negate";
        occ_yaml_raw << YAML::Value << 0;
        occ_yaml_raw << YAML::EndMap;
        occ_out_yaml_raw.close();
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

        /// write pgm headers
        occ_out_pgm << "P5 \n";
        occ_out_pgm << "# CREATOR: cslibs_mapping " << occ_resolution << "m/pix \n";
        occ_out_pgm << occ_width << " " << occ_height << "\n";
        occ_out_pgm << 255 << "\n";
        occ_out_raw_pgm << "P5 \n";
        occ_out_raw_pgm << "# CREATOR: cslibs_mapping " << occ_resolution << "m/pix \n";
        occ_out_raw_pgm << occ_width << " " << occ_height << "\n";
        occ_out_raw_pgm << 255 << "\n";

        auto convert_ros = [&occ_free_threshold, &occ_occupied_threshold](const T p) {
            if (cslibs_math::common::le(p, occ_free_threshold))
                return static_cast<uint8_t>(254);
            if (cslibs_math::common::le(p,occ_occupied_threshold))
                return static_cast<uint8_t>(0);
            return static_cast<uint8_t>(205);
        };
        auto convert_raw = [](const T p) {
            return static_cast<uint8_t>((1.0 - p) * 255);
        };

        for (std::size_t i = occ_height ; i > 0 ; --i) {
            for (std::size_t j = 0 ; j < occ_width ; ++j) {
                const T & data = occ_data[(i - 1) * occ_width + j];
                occ_out_pgm << convert_ros(data);
                occ_out_raw_pgm << convert_raw(data);
            }
        }

        occ_out_pgm.close();
        occ_out_raw_pgm.close();
    }

    // map data
    {
        const Tp occ_inv_resolution = 1.0 / occ_resolution;
        const cslibs_math_2d::Transform2<Tp> m_t_w = occ_origin.inverse();

        std::ofstream poses_out_yaml(poses_path_yaml);
        if (!poses_out_yaml.is_open()) {
            std::cout << "[SaveMap]: Could not open file '" << poses_path_yaml << "'." << std::endl;
            return false;
        }

        if (poses) {
            YAML::Emitter poses_yaml(poses_out_yaml);
            poses_yaml << YAML::BeginSeq;
            for (const auto &p_w : poses->poses) {
                const cslibs_math_2d::Transform2<Tp> p_m = m_t_w * cslibs_math_ros::geometry_msgs::conversion_2d::from<Tp>(p_w.pose);
                poses_yaml << YAML::Flow << YAML::BeginSeq << p_m.tx() * occ_inv_resolution << p_m.ty() * occ_inv_resolution << YAML::EndSeq;
            }
            poses_yaml << YAML::EndSeq;
            poses_out_yaml.close();
        }
    }

    return true;
}

inline bool savePath(
        const std::string    & poses_path_3d_yaml,
        const nav_msgs::Path & poses_path)
{
    std::ofstream poses_out_3d_yaml(poses_path_3d_yaml);
    if (!poses_out_3d_yaml.is_open()) {
        std::cout << "[SaveMap]: Could not open file '" << poses_path_3d_yaml << "'." << std::endl;
        return false;
    }

    YAML::Emitter poses_yaml(poses_out_3d_yaml);
    poses_yaml << YAML::BeginSeq;
    for (const auto &p_w : poses_path.poses)
        poses_yaml << YAML::Flow << YAML::BeginSeq
                   << p_w.pose.position.x << p_w.pose.position.y << p_w.pose.position.z
                   << p_w.pose.orientation.x << p_w.pose.orientation.y << p_w.pose.orientation.z
                   << p_w.pose.orientation.w << YAML::EndSeq;
    poses_yaml << YAML::EndSeq;
    poses_out_3d_yaml.close();

    return true;
}
}
}

#endif // CSLIBS_MAPPING_SAVE_MAP_HPP
