#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_2D_H
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_2d.hpp>

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>

#include <cslibs_ndt_2d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>

namespace cslibs_mapping {
namespace mapper {
template <typename T = double>
class OccupancyNDTGridMapper2DBase : public Mapper
{
public:
    using rep_t = maps::OccupancyNDTGridMap2D<T>;
    using ivm_t = cslibs_gridmaps::utility::InverseModel<T>;

    virtual const inline map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    inline void setupVisibilityBasedUpdateParameters(ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        visibility_based_update_ = nh.param<bool>(param_name("visibility_based_update"), false);

        const double prob_prior    = nh.param(param_name("prob_prior"), 0.5);
        const double prob_free     = nh.param(param_name("prob_free"), 0.45);
        const double prob_occupied = nh.param(param_name("prob_occupied"), 0.65);
        ivm_.reset(new ivm_t(
                       prob_prior, prob_free, prob_occupied));

        if (!visibility_based_update_)
            return;
        double visibility_threshold         = nh.param<double>("visibility_threshold", 0.4);
        double prob_visible_if_occluded     = nh.param<double>("prob_visible_if_occluded", 0.2);
        double prob_visible_if_not_occluded = nh.param<double>("prob_visible_if_not_occluded", 0.8);
        ivm_visibility_.reset(new ivm_t(
                                  visibility_threshold, prob_visible_if_occluded, prob_visible_if_not_occluded));

    }

    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const T resolution   = static_cast<T>(nh.param<double>(param_name("resolution"), 1.0));
        sampling_resolution_ = static_cast<T>(nh.param<double>(param_name("sampling_resolution"), (resolution / 40.0)));

        std::vector<double> origin = {0.0, 0.0, 0.0};
        origin = nh.param<std::vector<double>>(param_name("origin"), origin);

        if (origin.size() != 3)
            return false;

        setupVisibilityBasedUpdateParameters(nh);
        map_.reset(new rep_t(
                       map_frame_,
                       cslibs_math_2d::Pose2<T>(origin[0], origin[1], origin[2]), resolution));
        return true;
    }

    virtual inline bool uses(const data_t::ConstPtr &type) override
    {
        return type->isType<cslibs_plugins_data::types::Laserscan2<T>>();
    }

    virtual inline void process(const data_t::ConstPtr &data) override
    {
        assert (uses(data));
        assert (!visibility_based_update_ || ivm_);
        assert (!visibility_based_update_ || ivm_visibility_);

        const cslibs_plugins_data::types::Laserscan2<T> &laser_data = data->as<cslibs_plugins_data::types::Laserscan2<T>>();

        cslibs_math_2d::Transform2<T> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 laser_data.frame(),
                                 ros::Time(laser_data.timeFrame().start.seconds()),
                                 o_T_d,
                                 tf_timeout_)) {

            const typename cslibs_plugins_data::types::Laserscan2<T>::rays_t &rays = laser_data.getRays();
            typename cslibs_math_2d::Pointcloud2<T>::Ptr cloud(new cslibs_math_2d::Pointcloud2<T>);

            for (const auto &ray : rays)
                if (ray.valid() && ray.end_point.isNormal())
                    cloud->insert(ray.end_point);

            visibility_based_update_ ?
                        map_->get()->insertVisible(cloud, o_T_d, ivm_, ivm_visibility_) :
                        map_->get()->insert(cloud, o_T_d);
        }
    }

    virtual inline bool saveMap() override
    {
        if (!map_) {
            std::cout << "[OccupancyNDTGridMapper2D '" << name_ << "']: No Map." << std::endl;
            return true;
        }

        std::cout << "[OccupancyNDTGridMapper2D '" << name_ << "']: Saving Map..." << std::endl;
        if (!checkPath()) {
            std::cout << "[OccupancyNDTGridMapper2D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
            return false;
        }

        typename cslibs_gridmaps::static_maps::ProbabilityGridmap<T,T>::Ptr tmp;
        {
            if (!cslibs_ndt_2d::dynamic_maps::saveBinary<T>(map_->get(), (path_ / boost::filesystem::path("map")).string()))
                return false;

            cslibs_ndt_2d::conversion::from(map_->get(), tmp, sampling_resolution_, ivm_);
            if (!tmp)
                return false;
        }

        //cslibs_gridmaps::static_maps::algorithms::normalize<double>(*tmp);
        if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                            tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {

            std::cout << "[OccupancyNDTGridMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
        return false;
    }

private:
    typename rep_t::Ptr map_;

    typename ivm_t::Ptr ivm_;
    typename ivm_t::Ptr ivm_visibility_;
    bool                visibility_based_update_;
    T                   sampling_resolution_;
};

using OccupancyNDTGridMapper2D   = OccupancyNDTGridMapper2DBase<double>;
using OccupancyNDTGridMapper2D_d = OccupancyNDTGridMapper2DBase<double>;
using OccupancyNDTGridMapper2D_f = OccupancyNDTGridMapper2DBase<float>;
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_2D_H
