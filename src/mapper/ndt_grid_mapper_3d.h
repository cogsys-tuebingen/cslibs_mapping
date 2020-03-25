#ifndef CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H
#define CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/ndt_grid_map_3d.hpp>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_ndt_3d/serialization/serialization.hpp>
#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_time/time.hpp>

namespace cis = cslibs_indexed_storage;

namespace cslibs_mapping {
namespace mapper {
template <cslibs_ndt::map::tags::option option_t = cslibs_ndt::map::tags::dynamic_map,
          typename T = double,
          template <typename, typename, typename...> class backend_t = cis::backend::simple::UnorderedMap>
class NDTGridMapper3DBase : public Mapper
{
public:
    using rep_t = maps::NDTGridMap3D<option_t,T,backend_t>;
    virtual const inline map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const T resolution = static_cast<T>(nh.param<double>(param_name("resolution"), 1.0));
        std::vector<double> origin = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        origin = nh.param<std::vector<double>>(param_name("origin"), origin);

        if (origin.size() != 6)
            return false;

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

        std::vector<int> size = {0, 0, 0};
        std::vector<int> min_bundle_index = {0, 0, 0};
        size = nh.param<std::vector<int>>(param_name("size"), size);
        min_bundle_index = nh.param<std::vector<int>>(param_name("min_bundle_index"), min_bundle_index);

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
                                 ros::Time(cloud_data.timeFrame().start.seconds()),
                                 o_T_d,
                                 tf_timeout_)) {
            if (const typename cslibs_math_3d::Pointcloud3<T>::ConstPtr &cloud = cloud_data.points())
                map_->get()->insert(cloud, o_T_d);
            return true;
        }
        return false;
    }

    virtual inline bool saveMap() override
    {
        if (!map_) {
            std::cout << "[NDTGridMapper3D '" << name_ << "']: No Map." << std::endl;
            return true;
        }

        std::cout << "[NDTGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
        if (!checkPath()) {
            std::cout << "[NDTGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
            return false;
        }

        using path_t = boost::filesystem::path;
        path_t path_root(path_);
        if (!cslibs_ndt::common::serialization::create_directory(path_root))
            return false;

        if (cslibs_ndt_3d::serialization::saveBinary(*(map_->get()), (path_ / boost::filesystem::path("map")).string())) {
            std::cout << "[NDTGridMapper3D '" << name_ << "']: Saved Map successfully." << std::endl;
            return true;
        }
        return false;
    }

private:
    typename rep_t::Ptr map_;
};

namespace tag = cslibs_ndt::map::tags;
namespace backend = cis::backend;

using NDTGridMapper3D          = NDTGridMapper3DBase<>;
using NDTGridMapper3D_d_array  = NDTGridMapper3DBase<tag::static_map,  double, backend::array::Array>;
using NDTGridMapper3D_d_kdtree = NDTGridMapper3DBase<tag::dynamic_map, double, backend::kdtree::KDTree>;
using NDTGridMapper3D_d_map    = NDTGridMapper3DBase<tag::dynamic_map, double, backend::simple::Map>;
using NDTGridMapper3D_d_umap   = NDTGridMapper3DBase<tag::dynamic_map, double, backend::simple::UnorderedMap>;
using NDTGridMapper3D_d_ucmap  = NDTGridMapper3DBase<tag::dynamic_map, double, backend::simple::UnorderedComponentMap>;
using NDTGridMapper3D_f_array  = NDTGridMapper3DBase<tag::static_map,  float, backend::array::Array>;
using NDTGridMapper3D_f_kdtree = NDTGridMapper3DBase<tag::dynamic_map, float, backend::kdtree::KDTree>;
using NDTGridMapper3D_f_map    = NDTGridMapper3DBase<tag::dynamic_map, float, backend::simple::Map>;
using NDTGridMapper3D_f_umap   = NDTGridMapper3DBase<tag::dynamic_map, float, backend::simple::UnorderedMap>;
using NDTGridMapper3D_f_ucmap  = NDTGridMapper3DBase<tag::dynamic_map, float, backend::simple::UnorderedComponentMap>;
}
}

#endif // CSLIBS_MAPPING_NDT_GRID_MAPPER_3D_H
