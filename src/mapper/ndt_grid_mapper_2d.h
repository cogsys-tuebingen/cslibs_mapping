#ifndef CSLIBS_MAPPING_NDT_GRID_MAPPER_2D_H
#define CSLIBS_MAPPING_NDT_GRID_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/ndt_grid_map_2d.hpp>

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>

namespace cslibs_mapping {
namespace mapper {
template <typename T>
class NDTGridMapper2DBase : public Mapper
{
public:
    using rep_t = maps::NDTGridMap2D<T>;
    virtual inline const map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const T resolution = static_cast<T>(nh.param<double>(param_name("resolution"), 1.0));
        sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), (resolution / 40.0));
        std::vector<double> origin = {0.0, 0.0, 0.0};
        nh.param<std::vector<double>>(param_name("origin"), origin);

        if (origin.size() != 3)
            return false;

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

        const cslibs_plugins_data::types::Laserscan2<T> &laser_data = data->as<cslibs_plugins_data::types::Laserscan2<T>>();

        cslibs_math_2d::Transform2<T> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 laser_data.frame(),
                                 ros::Time(laser_data.timeFrame().start.seconds()),
                                 o_T_d,
                                 tf_timeout_)) {

            const typename cslibs_plugins_data::types::Laserscan2<T>::rays_t rays = laser_data.getRays();
            typename cslibs_math_2d::Pointcloud2<T>::Ptr cloud(new cslibs_math_2d::Pointcloud2<T>);

            for (const auto &ray : rays)
                if (ray.valid() && ray.end_point.isNormal())
                    cloud->insert(ray.end_point);

            map_->get()->insert(cloud, o_T_d);
        }
    }

    virtual inline bool saveMap() override
    {
        if (!map_) {
            std::cout << "[NDTGridMapper2D '" << name_ << "']: No Map." << std::endl;
            return true;
        }

        std::cout << "[NDTGridMapper2D '" << name_ << "']: Saving Map..." << std::endl;
        if (!checkPath()) {
            std::cout << "[NDTGridMapper2D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
            return false;
        }

        typename cslibs_gridmaps::static_maps::ProbabilityGridmap<T,T>::Ptr tmp;
        {
            if (!cslibs_ndt_2d::dynamic_maps::saveBinary<T>(map_->get(), (path_ / boost::filesystem::path("map")).string()))
                return false;

            cslibs_ndt_2d::conversion::from(map_->get(), tmp, sampling_resolution_);
            if (!tmp)
                return false;
        }

        //cslibs_gridmaps::static_maps::algorithms::normalize<double>(*tmp);
        if (cslibs_mapping::mapper::saveMap(path_, nullptr, tmp->getData(), tmp->getHeight(),
                                            tmp->getWidth(), tmp->getOrigin(), tmp->getResolution())) {

            std::cout << "[NDTGridMapper2D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
        return false;
    }

private:
    typename rep_t::Ptr map_;
    T                   sampling_resolution_;
};

using NDTGridMapper2D   = NDTGridMapper2DBase<double>;
using NDTGridMapper2D_d = NDTGridMapper2DBase<double>;
using NDTGridMapper2D_f = NDTGridMapper2DBase<float>;
}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAPPER_2D_H
