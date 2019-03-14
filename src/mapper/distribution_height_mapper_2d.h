#ifndef CSLIBS_MAPPING_DISTRIBUTION_HEIGHT_MAPPER_2D_H
#define CSLIBS_MAPPING_DISTRIBUTION_HEIGHT_MAPPER_2D_H

#include <mutex>
#include <atomic>
#include <condition_variable>

#include <cslibs_mapping/mapper/mapper.hpp>
#include <cslibs_mapping/maps/distribution_height_map_2d.hpp>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_gridmaps/static_maps/algorithms/normalize.hpp>
#include <cslibs_gridmaps/static_maps/probability_gridmap.h>
#include <cslibs_gridmaps/serialization/dynamic_maps/distribution_heightmap.hpp>

namespace cslibs_mapping {
namespace mapper {
template <typename Tp = double, typename T = double>
class DistributionHeightMapper2DBase : public Mapper
{
public:
    using rep_t = maps::DistributionHeightMap2D<Tp,T>;
    virtual const inline map_t::ConstPtr getMap() const override
    {
        return map_;
    }

protected:
    virtual inline bool setupMap(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const double resolution       = nh.param<double>(param_name("resolution"), 0.05);
        const double chunk_resolution = nh.param<double>(param_name("chunk_resolution"), 5.0);
        const double max_height       = nh.param<double>(param_name("max_height"), 1.0);

        std::vector<double> origin = {0.0, 0.0, 0.0};
        nh.param<std::vector<double>>(param_name("origin"), origin);
        if (origin.size() != 3)
            return false;

        const cslibs_math_2d::Pose2<Tp> origin_pose(origin[0], origin[1], origin[2]);
        save_all_ = nh.param<bool>(param_name("save_all"), false);
        map_.reset(new rep_t(map_frame_, origin_pose, resolution, chunk_resolution, max_height));
        return true;
    }

    virtual inline bool uses(const data_t::ConstPtr &type) override
    {
        return type->isType<cslibs_plugins_data::types::Pointcloud3<Tp>>();
    }

    virtual inline void process(const data_t::ConstPtr &data) override
    {
        assert (uses(data));

        const cslibs_plugins_data::types::Pointcloud3<Tp> &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud3<Tp>>();
        const typename cslibs_gridmaps::dynamic_maps::DistributionHeightmap<Tp,T>::Ptr &map = map_->get();

        cslibs_math_3d::Transform3<Tp> o_T_d;
        if (tf_->lookupTransform(map_frame_,
                                 cloud_data.frame(),
                                 ros::Time(cloud_data.timeFrame().start.seconds()),
                                 o_T_d,
                                 tf_timeout_)) {
            const cslibs_math_2d::Point2<Tp> sensor_xy(o_T_d.tx(), o_T_d.ty());
            const double &sensor_z = o_T_d.tz();

            const typename cslibs_math_3d::Pointcloud3<Tp>::ConstPtr &points = cloud_data.points();
            if (points) {
                for (const auto &point : *points) {
                    if (point.isNormal()) {
                        const cslibs_math_3d::Point3<Tp> map_point = o_T_d * point;
                        if (map_point.isNormal())
                            map->insert(sensor_xy, sensor_z,
                                        cslibs_math_2d::Point2<Tp>(map_point(0), map_point(1)), map_point(2));
                    }
                }
            }
        }
    }

    virtual inline bool saveMap() override
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

        if (save_all_) {
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
        }

        typename cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>::Ptr tmp;
        {
            const typename cslibs_gridmaps::dynamic_maps::DistributionHeightmap<Tp,T>::Ptr &map = map_->get();
            tmp.reset(new cslibs_gridmaps::static_maps::ProbabilityGridmap<Tp,T>(map->getOrigin(),
                                                                                 map->getResolution(),
                                                                                 map->getHeight(),
                                                                                 map->getWidth()));
            const std::size_t chunk_step = map->getChunkSize();
            const std::array<int, 2> min_chunk_index = map->getMinChunkIndex();
            const std::array<int, 2> max_chunk_index = map->getMaxChunkIndex();

            for(int i = min_chunk_index[1] ; i <= max_chunk_index[1] ; ++ i) {
                for(int j = min_chunk_index[0] ; j <= max_chunk_index[0] ; ++ j) {
                    typename cslibs_gridmaps::dynamic_maps::DistributionHeightmap<Tp,T>::chunk_t *chunk = map->getChunk({{j,i}});
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

private:
    typename rep_t::Ptr map_;
    bool                save_all_;
};

using DistributionHeightMapper2D = DistributionHeightMapper2DBase<double,double>;
}
}

#endif // CSLIBS_MAPPING_DISTRIBUTION_HEIGHT_MAPPER_2D_H
