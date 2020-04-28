#ifndef CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.hpp>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_ndt_3d/serialization/serialization.hpp>
#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>

#include <cslibs_time/time.hpp>
#include <cslibs_math/statistics/stable_distribution.hpp>

namespace cslibs_mapping {
namespace mapper {
template <cslibs_ndt::map::tags::option option_t = cslibs_ndt::map::tags::dynamic_map,
          typename T = double,
          template <typename, typename, typename...> class backend_t = cis::backend::simple::UnorderedMap>
class OccupancyNDTGridMapper3DBase : public Mapper
{
public:
    using rep_t = maps::OccupancyNDTGridMap3D<option_t,T,backend_t>;
    using ivm_t = cslibs_gridmaps::utility::InverseModel<T>;
    virtual ~OccupancyNDTGridMapper3DBase()
    {
        std::string stats_print =
                "[OccupancyNDTGridMapper3D]: N | mean | std | mem = " +
                std::to_string(stats_.getN())
                + " | " + std::to_string(stats_.getMean())
                + " | " + std::to_string(stats_.getStandardDeviation())
                /*+ " | " + std::to_string(map_->get()->getByteSize())*/ + "\n";
        std::cout << stats_print << std::endl;

        std::vector<std::array<int,3>> indices;
        cslibs_math::statistics::StableDistribution<double,1,6> traversal;
        for (int i=0; i<50; ++i) {
            indices.clear();
            cslibs_time::Time now = cslibs_time::Time::now();
            map_->get()->getBundleIndices(indices);
            const double time = (cslibs_time::Time::now() - now).milliseconds();
            traversal += time;
        }
        std::cout << "[OccupancyNDTGridMapper3D]: traversal N | mean | std = \n"
                  << std::to_string(traversal.getN())
                  << " | " << std::to_string(traversal.getMean())
                  << " | " << std::to_string(traversal.getStandardDeviation())
                  <<" || " << std::to_string(traversal.getMean() / indices.size())
                  << " | " << std::to_string(traversal.getStandardDeviation() / indices.size())
                  << std::endl;

        std::vector<typename rep_t::map_t::distribution_bundle_t*> vec;
        cslibs_math::statistics::StableDistribution<double,1,6> access;
        for (auto &index : indices) {
            cslibs_time::Time now = cslibs_time::Time::now();
            vec.push_back(map_->get()->getDistributionBundle(index));
            const double time = (cslibs_time::Time::now() - now).milliseconds();
            access += time;
        }
        std::cout << "[OccupancyNDTGridMapper3D]: access N | mean | std = \n"
                  << std::to_string(access.getN())
                  << " | " << std::to_string(access.getMean())
                  << " | " << std::to_string(access.getStandardDeviation())
                  << std::endl;

        std::cout << "[OccupancyNDTGridMapper3D]: byte_size = " << std::to_string(map_->get()->getByteSize()) << std::endl;
    }

    virtual const inline map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    inline void setupVisibilityBasedUpdateParameters(ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        visibility_based_update_ = nh.param<bool>(param_name("visibility_based_update"), false);

        const T prob_prior    = static_cast<T>(nh.param(param_name("prob_prior"),    0.5));
        const T prob_free     = static_cast<T>(nh.param(param_name("prob_free"),     0.45));
        const T prob_occupied = static_cast<T>(nh.param(param_name("prob_occupied"), 0.65));
        ivm_.reset(new ivm_t(
                       prob_prior, prob_free, prob_occupied));

        if (!visibility_based_update_)
            return;
        T visibility_threshold         = static_cast<T>(nh.param<double>(param_name("visibility_threshold"), 0.05));
        T prob_visible_if_occluded     = static_cast<T>(nh.param<double>(param_name("prob_visible_if_occluded"), 0.2));
        T prob_visible_if_not_occluded = static_cast<T>(nh.param<double>(param_name("prob_visible_if_not_occluded"), 0.8));
        ivm_visibility_.reset(new ivm_t(
                                  visibility_threshold, prob_visible_if_occluded, prob_visible_if_not_occluded));

    }

    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const T resolution = static_cast<T>(nh.param<double>(param_name("resolution"), 1.0));
        std::vector<double> origin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        origin = nh.param<std::vector<double>>(param_name("origin"), origin);

        if (origin.size() != 6)
            return false;

        setupVisibilityBasedUpdateParameters(nh);
        setupMap(nh, origin, resolution);
        return true;
    }

    template<cslibs_ndt::map::tags::option O = option_t>
    inline typename std::enable_if<O == cslibs_ndt::map::tags::dynamic_map, void>::type
    setupMap(ros::NodeHandle &nh, const std::vector<double>& origin, const T& resolution)
    {
        map_.reset(new rep_t(
                       map_frame_,
                       cslibs_math_3d::Pose3<T>(
                           cslibs_math_3d::Vector3<T>(static_cast<T>(origin[0]), static_cast<T>(origin[1]), static_cast<T>(origin[2])),
                           cslibs_math_3d::Quaternion<T>(static_cast<T>(origin[3]), static_cast<T>(origin[4]), static_cast<T>(origin[5]))),
                       resolution));
    }

    template<cslibs_ndt::map::tags::option O = option_t>
    inline typename std::enable_if<O == cslibs_ndt::map::tags::static_map, void>::type
    setupMap(ros::NodeHandle &nh, const std::vector<double>& origin, const T& resolution)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        std::vector<double> extents   = {0.,0.,0.};
        std::vector<double> min_point = {0.,0.,0.};
        extents = nh.param<std::vector<double>>(param_name("extents"), extents);
        min_point = nh.param<std::vector<double>>(param_name("min_point"), min_point);

        std::vector<int> size = {static_cast<int>(std::ceil(extents[0]/resolution)),
                                 static_cast<int>(std::ceil(extents[1]/resolution)),
                                 static_cast<int>(std::ceil(extents[2]/resolution))};
        std::vector<int> min_bundle_index = {static_cast<int>(std::floor(min_point[0]/resolution)),
                                             static_cast<int>(std::floor(min_point[1]/resolution)),
                                             static_cast<int>(std::floor(min_point[2]/resolution))};

        map_.reset(new rep_t(
                       map_frame_,
                       cslibs_math_3d::Pose3<T>(
                           cslibs_math_3d::Vector3<T>(static_cast<T>(origin[0]), static_cast<T>(origin[1]), static_cast<T>(origin[2])),
                           cslibs_math_3d::Quaternion<T>(static_cast<T>(origin[3]), static_cast<T>(origin[4]), static_cast<T>(origin[5]))),
                       resolution,
                       std::array<std::size_t,3>{static_cast<std::size_t>(size[0]), static_cast<std::size_t>(size[1]), static_cast<std::size_t>(size[2])},
                       std::array<int,3>{min_bundle_index[0], min_bundle_index[1], min_bundle_index[2]}));
    }

    virtual inline bool uses(const data_t::ConstPtr &type) override
    {
        return type->isType<cslibs_plugins_data::types::Pointcloud3<T>>();
    }

    virtual inline bool process(const data_t::ConstPtr &data) override
    {
        assert (uses(data));

        const cslibs_plugins_data::types::Pointcloud3<T> &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud3<T>>();

        cslibs_math_3d::Transform3<T> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 cloud_data.frame(),
                                 ros::Time().fromNSec(cloud_data.timeFrame().start.nanoseconds()),
                                 o_T_d,
                                 tf_timeout_)) {
            using iterator_t = cslibs_math_3d::algorithms::NDTIterator<T>;
            if (const typename cslibs_math_3d::Pointcloud3<T>::ConstPtr &cloud = cloud_data.points()) {

                const cslibs_time::Time start = cslibs_time::Time::now();
                visibility_based_update_ ?
                            map_->get()->template insertVisible<iterator_t>(cloud, o_T_d, ivm_, ivm_visibility_) :
                            map_->get()->template insert<iterator_t>(cloud, o_T_d);
                const double time = (cslibs_time::Time::now() - start).milliseconds();
                stats_ += time;

                std::cout << "[OccupancyNDTGridMapper3D]: N = " << stats_.getN() << std::endl;
                return true;
            }
        }
        return false;
    }

    virtual inline bool saveMap() override
    {
        if (!map_) {
            std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: No Map." << std::endl;
            return true;
        }

        std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
        if (!checkPath()) {
            std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
            return false;
        }

        using path_t = boost::filesystem::path;
        path_t path_root(path_);
        if (!cslibs_ndt::common::serialization::create_directory(path_root))
            return false;

        std::ofstream out((path_root / path_t("stats")).string(), std::fstream::trunc);
        std::string stats_print =
                "[OccupancyNDTGridMapper3D]: N | mean | std | mem = " +
                std::to_string(stats_.getN())
                + " | " + std::to_string(stats_.getMean())
                + " | " + std::to_string(stats_.getStandardDeviation())
                /*+ " | " + std::to_string(map_->get()->getByteSize())*/ + "\n";
        std::cout << stats_print << std::endl;
        out << stats_print << std::endl;
        out.close();

        if (cslibs_ndt_3d::serialization::saveBinary(*(map_->get()), (path_ / boost::filesystem::path("map")).string())) {
            std::cout << "[OccupancyNDTGridMapper3D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
        return false;
    }

private:
    typename rep_t::Ptr map_;

    typename ivm_t::Ptr ivm_;
    typename ivm_t::Ptr ivm_visibility_;
    bool                visibility_based_update_;

    cslibs_math::statistics::StableDistribution<double,1,6> stats_;
};

namespace tag = cslibs_ndt::map::tags;
namespace backend = cis::backend;

using OccupancyNDTGridMapper3D          = OccupancyNDTGridMapper3DBase<>;
using OccupancyNDTGridMapper3D_d_array  = OccupancyNDTGridMapper3DBase<tag::static_map,  double, backend::array::Array>;
using OccupancyNDTGridMapper3D_d_kdtree = OccupancyNDTGridMapper3DBase<tag::dynamic_map, double, backend::kdtree::KDTree>;
using OccupancyNDTGridMapper3D_d_map    = OccupancyNDTGridMapper3DBase<tag::dynamic_map, double, backend::simple::Map>;
using OccupancyNDTGridMapper3D_d_umap   = OccupancyNDTGridMapper3DBase<tag::dynamic_map, double, backend::simple::UnorderedMap>;
using OccupancyNDTGridMapper3D_d_ucmap  = OccupancyNDTGridMapper3DBase<tag::dynamic_map, double, backend::simple::UnorderedComponentMap>;
using OccupancyNDTGridMapper3D_f_array  = OccupancyNDTGridMapper3DBase<tag::static_map,  float, backend::array::Array>;
using OccupancyNDTGridMapper3D_f_kdtree = OccupancyNDTGridMapper3DBase<tag::dynamic_map, float, backend::kdtree::KDTree>;
using OccupancyNDTGridMapper3D_f_map    = OccupancyNDTGridMapper3DBase<tag::dynamic_map, float, backend::simple::Map>;
using OccupancyNDTGridMapper3D_f_umap   = OccupancyNDTGridMapper3DBase<tag::dynamic_map, float, backend::simple::UnorderedMap>;
using OccupancyNDTGridMapper3D_f_ucmap  = OccupancyNDTGridMapper3DBase<tag::dynamic_map, float, backend::simple::UnorderedComponentMap>;
}
}

#endif // CSLIBS_MAPPING_OCCUPANCY_NDT_GRID_MAPPER_3D_H
