#ifndef CSLIBS_MAPPING_OCCUPANCY_GRID_MAPPER_2D_H
#define CSLIBS_MAPPING_OCCUPANCY_GRID_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/mapper/save_map.hpp>
#include <cslibs_mapping/maps/occupancy_grid_map_2d.hpp>

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>

#include <cslibs_gridmaps/utility/inverse_model.hpp>

#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <cslibs_gridmaps/serialization/dynamic_maps/probability_gridmap.hpp>

namespace cslibs_mapping {
namespace mapper {
template <typename Tp = double, typename T = double>
class OccupancyGridMapper2DBase : public Mapper
{
public:
    using rep_t = cslibs_mapping::maps::OccupancyGridMap2D<Tp,T>;
    using ivm_t = cslibs_gridmaps::utility::InverseModel<T>;

    virtual const inline map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const Tp resolution       = static_cast<Tp>(nh.param<double>(param_name("resolution"), 0.05));
        const Tp chunk_resolution = static_cast<Tp>(nh.param<double>(param_name("chunk_resolution"), 5.0));
        resolution_ = resolution;

        std::vector<double> origin = {0.0, 0.0, 0.0};
        nh.param<std::vector<double>>(param_name("origin"), origin);

        const T prob_prior     = static_cast<T>(nh.param(param_name("prob_prior"),    0.5));
        const T prob_free      = static_cast<T>(nh.param(param_name("prob_free"),     0.45));
        const T prob_occupied  = static_cast<T>(nh.param(param_name("prob_occupied"), 0.65));
        ivm_.reset(new ivm_t(prob_prior, prob_free, prob_occupied));

        if (origin.size() != 3 || !ivm_)
            return false;

        map_.reset(new rep_t(
                       map_frame_,
                       cslibs_math_2d::Pose2<Tp>(static_cast<Tp>(origin[0]), static_cast<Tp>(origin[1]), static_cast<Tp>(origin[2])), resolution, chunk_resolution));
        return true;
    }

    virtual inline bool uses(const data_t::ConstPtr &type) override
    {
        return type->isType<cslibs_plugins_data::types::Laserscan2<Tp>>();
    }

    virtual inline bool process(const data_t::ConstPtr &data) override
    {
        assert (uses(data));
        assert (ivm_);

        const cslibs_plugins_data::types::Laserscan2<Tp> laser_data = data->as<cslibs_plugins_data::types::Laserscan2<Tp>>();

        cslibs_math_2d::Transform2<Tp> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 laser_data.frame(),
                                 ros::Time().fromNSec(laser_data.timeFrame().start.nanoseconds()),
                                 o_T_d,
                                 tf_timeout_)) {

            const typename cslibs_plugins_data::types::Laserscan2<Tp>::rays_t &rays = laser_data.getRays();
            const typename cslibs_gridmaps::dynamic_maps::ProbabilityGridmap<Tp,T>::Ptr &map = map_->get();

            const auto m_T_w = map->getInitialOrigin().inverse();
            const cslibs_math_2d::Point2<Tp> start = m_T_w * o_T_d.translation();
            for (const auto &ray : rays) {
                if (ray.valid() && ray.end_point.isNormal()) {
                    const cslibs_math_2d::Point2<Tp> map_point = o_T_d * ray.end_point;
                    if (map_point.isNormal()) {
                        const Tp& range = (m_T_w * map_point - start).length();

                        auto it = map->getLineIterator(o_T_d.translation(), map_point);
                        while(!it.done()) {
                            const cslibs_math_2d::Point2<Tp> mid((static_cast<Tp>(it.x())+0.5) * resolution_,
                                                                 (static_cast<Tp>(it.y())+0.5) * resolution_);
                            *it = (2.0*std::fabs(range - (mid-start).length()) > resolution_) ?
                                       ivm_->updateFree(*it) : ivm_->updateOccupied(*it);
                            ++it;
                        }
                        *it = ivm_->updateOccupied(*it);
                    }
                }
            }
            return true;
        }
        return false;
    }

    virtual inline bool saveMap() override
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

        typename cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>::Ptr tmp;
        {
            const typename cslibs_gridmaps::dynamic_maps::ProbabilityGridmap<Tp,T>::Ptr &map = map_->get();
            tmp.reset(new cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>(map->getOrigin(),
                                                                                 map->getResolution(),
                                                                                 map->getHeight(),
                                                                                 map->getWidth(),
                                                                                 ivm_->getLogOddsPrior()));
            const std::size_t chunk_step = map->getChunkSize();
            const std::array<int, 2> min_chunk_index = map->getMinChunkIndex();
            const std::array<int, 2> max_chunk_index = map->getMaxChunkIndex();
            for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
                for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {
                    typename cslibs_gridmaps::dynamic_maps::ProbabilityGridmap<Tp,T>::chunk_t *chunk = map->getChunk({{j,i}});
                    if (chunk) {
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
            cslibs_gridmaps::static_maps::conversion::LogOdds::from<Tp,T>(tmp, tmp);
            if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                                tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {
                std::cout << "[OccupancyGridMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
                return true;
            }
        }
        return false;
    }

private:
    typename rep_t::Ptr map_;

    typename ivm_t::Ptr ivm_;
    Tp                  resolution_;
};

using OccupancyGridMapper2D    = OccupancyGridMapper2DBase<double,double>; // for backwards compatibility
using OccupancyGridMapper2D_dd = OccupancyGridMapper2DBase<double,double>;
using OccupancyGridMapper2D_df = OccupancyGridMapper2DBase<double,float>;
using OccupancyGridMapper2D_ff = OccupancyGridMapper2DBase<float,float>;
using OccupancyGridMapper2D_fd = OccupancyGridMapper2DBase<float,double>;
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_GRID_MAPPER_2D_H
