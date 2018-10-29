#include "occupancy_grid_mapper_2d.h"

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>

#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_gridmaps/serialization/dynamic_maps/probability_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::OccupancyGridMapper2D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const OccupancyGridMapper2D::map_t::ConstPtr OccupancyGridMapper2D::getMap() const
{
    return map_;
}

bool OccupancyGridMapper2D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution       = nh.param<double>(param_name("resolution"), 0.05);
    const double chunk_resolution = nh.param<double>(param_name("chunk_resolution"), 5.0);
    resolution2_ = resolution * resolution;

    std::vector<double> origin = {0.0, 0.0, 0.0};
    nh.param<std::vector<double>>(param_name("origin"), origin);

    const double prob_prior     = nh.param(param_name("prob_prior"),    0.5);
    const double prob_free      = nh.param(param_name("prob_free"),     0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    ivm_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));

    if (origin.size() != 3 || !ivm_)
        return false;

    map_.reset(new maps::OccupancyGridMap2D(
                   map_frame_,
                   cslibs_math_2d::Pose2d(origin[0], origin[1], origin[2]), resolution, chunk_resolution));
    return true;
}

bool OccupancyGridMapper2D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Laserscan>();
}

void OccupancyGridMapper2D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));
    assert (ivm_);

    const cslibs_plugins_data::types::Laserscan laser_data = data->as<cslibs_plugins_data::types::Laserscan>();

    cslibs_math_2d::Transform2d o_T_d;
    if (tf_->lookupTransform(map_frame_,
                             laser_data.getFrame(),
                             ros::Time(laser_data.getTimeFrame().start.seconds()),
                             o_T_d,
                             tf_timeout_)) {

        const cslibs_plugins_data::types::Laserscan::rays_t &rays = laser_data.getRays();
        const cslibs_gridmaps::dynamic_maps::ProbabilityGridmap::Ptr &map = map_->get();

        for (const auto &ray : rays) {
            if (ray.valid() && ray.end_point.isNormal()) {
                const cslibs_math_2d::Point2d map_point = o_T_d * ray.end_point;
                if (map_point.isNormal()) {
                    auto it = map->getLineIterator(o_T_d.translation(), map_point);
                    while(!it.done()) {
                        double l = it.distance2() > resolution2_ ?
                                    ivm_->updateFree(*it) : ivm_->updateOccupied(*it);
                        *it = l;
                        ++it;
                    }
                    *it = ivm_->updateOccupied(*it);
                }
            }
        }
    }
}

bool OccupancyGridMapper2D::saveMap()
{
    if (!map_) {
        std::cout << "[OccupancyGridMapper2D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[OccupancyGridMapper2D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[OccupancyGridMapper2D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    std::string map_path_yaml = (path_ / boost::filesystem::path("map.yaml")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if (!map_out_yaml.is_open()) {
            std::cout << "[OccupancyGridMapper2D '" << name_ << "']: Could not open file '" << map_path_yaml << "'." << std::endl;
            return false;
        }
        map_out_yaml << YAML::Node(map_->get());
        map_out_yaml.close();
    }

    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr tmp;
    {
        const cslibs_gridmaps::dynamic_maps::ProbabilityGridmap::Ptr &map = map_->get();
        tmp.reset(new cslibs_gridmaps::static_maps::ProbabilityGridmap(map->getOrigin(),
                                                                       map->getResolution(),
                                                                       map->getHeight(),
                                                                       map->getWidth(),
                                                                       ivm_->getLogOddsPrior()));
        const std::size_t chunk_step = map->getChunkSize();
        const std::array<int, 2> min_chunk_index = map->getMinChunkIndex();
        const std::array<int, 2> max_chunk_index = map->getMaxChunkIndex();
        for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
            for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {
                cslibs_gridmaps::dynamic_maps::ProbabilityGridmap::chunk_t *chunk = map->getChunk({{j,i}});
                if (chunk != nullptr) {
                    const std::size_t cx = static_cast<std::size_t>((j - min_chunk_index[0]) * static_cast<int>(chunk_step));
                    const std::size_t cy = static_cast<std::size_t>((i - min_chunk_index[1]) * static_cast<int>(chunk_step));
                    for (std::size_t k = 0 ; k < chunk_step ; ++ k)
                        for (std::size_t l = 0 ; l < chunk_step ; ++ l)
                            tmp->at(cx + l, cy + k) = chunk->at(l,k);
                }
            }
        }
    }

    if (tmp) {
        cslibs_gridmaps::static_maps::conversion::LogOdds::from(tmp, tmp);
        if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                            tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {

            std::cout << "[OccupancyGridMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
    }
    return false;
}
}
}
