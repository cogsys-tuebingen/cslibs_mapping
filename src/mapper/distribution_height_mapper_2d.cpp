#include "distribution_height_mapper_2d.h"

#include <cslibs_plugins_data/types/pointcloud.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>
#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/serialization/dynamic_maps/distribution_heightmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::DistributionHeightMapper2D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const DistributionHeightMapper2D::map_t::ConstPtr DistributionHeightMapper2D::getMap() const
{
    return map_;
}

bool DistributionHeightMapper2D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution       = nh.param<double>(param_name("resolution"), 0.05);
    const double chunk_resolution = nh.param<double>(param_name("chunk_resolution"), 5.0);
    const double max_height       = nh.param<double>(param_name("max_height"), 1.0);

    std::vector<double> origin = {0.0, 0.0, 0.0};
    nh.param<std::vector<double>>(param_name("origin"), origin);
    if (origin.size() != 3)
        return false;

    const cslibs_math_2d::Pose2d origin_pose(origin[0], origin[1], origin[2]);
    map_.reset(new maps::DistributionHeightMap2D(map_frame_, origin_pose, resolution, chunk_resolution, max_height));
    return true;
}

bool DistributionHeightMapper2D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud>();
}

void DistributionHeightMapper2D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));

    const cslibs_plugins_data::types::Pointcloud &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud>();
    const cslibs_gridmaps::dynamic_maps::DistributionHeightmap::Ptr &map = map_->get();

    tf::Transform o_T_d_tmp;
    if (tf_->lookupTransform(map_frame_,
                             cloud_data.getFrame(),
                             ros::Time(cloud_data.getTimeFrame().start.seconds()),
                             o_T_d_tmp,
                             tf_timeout_)) {
        cslibs_math_3d::Transform3d o_T_d = cslibs_math_ros::tf::conversion_3d::from(o_T_d_tmp);
        const cslibs_math_2d::Point2d sensor_xy(o_T_d.tx(), o_T_d.ty());
        const double &sensor_z = o_T_d.tz();

        const cslibs_math_3d::Pointcloud3d::Ptr &points = cloud_data.getPoints();
        if (points) {
            for (const auto &point : *points) {
                if (point.isNormal()) {
                    const cslibs_math_3d::Point3d map_point = o_T_d * point;
                    if (map_point.isNormal())
                        map->insert(sensor_xy, sensor_z,
                                    cslibs_math_2d::Point2d(map_point(0), map_point(1)), map_point(2));
                }
            }
        }
    }
}

bool DistributionHeightMapper2D::saveMap()
{
    if (!map_) {
        std::cout << "[DistributionHeightMapper2D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[DistributionHeightMapper2D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[DistributionHeightMapper2D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    std::string map_path_yaml = (path_ / boost::filesystem::path("map.yaml")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if (!map_out_yaml.is_open()) {
            std::cout << "[DistributionHeightMapper2D '" << name_ << "']: Could not open file '" << map_path_yaml << "'." << std::endl;
            return false;
        }
        map_out_yaml << YAML::Node(map_->get());
        map_out_yaml.close();
    }

    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr tmp;
    {
        const cslibs_gridmaps::dynamic_maps::DistributionHeightmap::Ptr &map = map_->get();
        tmp.reset(new cslibs_gridmaps::static_maps::ProbabilityGridmap(map->getOrigin(),
                                                                       map->getResolution(),
                                                                       map->getHeight(),
                                                                       map->getWidth()));
        const std::size_t chunk_step = map->getChunkSize();
        const std::array<int, 2> min_chunk_index = map->getMinChunkIndex();
        const std::array<int, 2> max_chunk_index = map->getMaxChunkIndex();

        for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
            for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {
                cslibs_gridmaps::dynamic_maps::DistributionHeightmap::chunk_t *chunk = map->getChunk({{j,i}});
                if (chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));
                    for (std::size_t k = 0 ; k < chunk_step ; ++ k) {
                        for (std::size_t l = 0 ; l < chunk_step ; ++ l){
                            const double &val = chunk->at(l, k).getMean();
                            tmp->at(cx + l, cy + k) = std::isnormal(val) ? val : 0.5;
                        }
                    }
                }
            }
        }
    }

    if (tmp) {
        if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                            tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {

            std::cout << "[DistributionHeightMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
    }
    return false;
}
}
}
