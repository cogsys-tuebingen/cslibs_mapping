#include "occupancy_grid_mapper_3d.h"

#include <cslibs_math_ros/tf/conversion_3d.hpp>
#include <cslibs_time/time.hpp>
#include <cslibs_ndt/serialization/filesystem.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::OccupancyGridMapper3D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
OccupancyGridMapper3D::~OccupancyGridMapper3D()
{
    double sx,sy,sz;
    map_->get()->getMetricSize(sx,sy,sz);
    std::string stats_print =
            "[OccupancyGridMapper3D]: N | mean | std | mem = " +
            std::to_string(stats_.getN())
            + " | " + std::to_string(stats_.getMean())
            + " | " + std::to_string(stats_.getStandardDeviation())
            + " | " + std::to_string(map_->get()->memoryUsage())
            + " | " + "[" + std::to_string(sx) + "," + std::to_string(sy) + "," + std::to_string(sz) + "]" + "\n";
    std::cout << stats_print << std::endl;
/*
    cslibs_math::statistics::StableDistribution<double,1,6> traversal;
    std::vector<octomap::OcTreeNode*> cells;
    double traversal_overall = 0.0;
    for (int i=0; i<iterations_; ++i) {
        cells.clear();
        cslibs_time::Time now = cslibs_time::Time::now();
        for (octomap::OcTree::leaf_iterator it = map_->get()->begin_leafs(), end = map_->get()->end_leafs() ; it != end ; ++ it)
            cells.emplace_back(&(*it));
        const double time = (cslibs_time::Time::now() - now).nanoseconds();
        traversal += time;
        traversal_overall += time;
    }

    std::vector<octomap::point3d> indices;
    for (octomap::OcTree::leaf_iterator it = map_->get()->begin_leafs(), end = map_->get()->end_leafs() ; it != end ; ++ it)
        indices.emplace_back(it.getCoordinate());
    std::cout << "[OccupancyGridMapper3D]: traversal N | mean | std [ms] || traversal normalized mean | std [ns] = \n"
              << std::to_string(traversal.getN())
              << " | " << std::to_string(traversal.getMean() * 1e-6)
              << " | " << std::to_string(traversal.getStandardDeviation() * 1e-6)
              <<" || " << std::to_string(traversal.getMean() / indices.size())
              << " | " << std::to_string(traversal.getStandardDeviation() / indices.size())
              << std::endl;
    std::cout << "[OccupancyGridMapper3D]: traversal mean [ms / ns] | traversal normalized mean [ns] = \n"
              << std::to_string(traversal_overall * 1e-6 / iterations_)
              << " / " << std::to_string(traversal_overall / iterations_)
              << " | " << std::to_string(traversal_overall / iterations_ / indices.size())
              << std::endl;
*/
    cslibs_math::statistics::StableDistribution<double,1,6> access;
    double access_overall = 0.0;
    double size = 0.0;
    for (int i=0; i<iterations_; ++i) {
//        cells.clear();
//    for (auto &index : indices) {
//        if (clear_)
//            cells.clear();
        size = 0.0;
        for (octomap::OcTree::leaf_iterator it = map_->get()->begin_leafs(), end = map_->get()->end_leafs() ; it != end ; ++ it) {
            const octomap::point3d& in = it.getCoordinate();

        cslibs_time::Time now = cslibs_time::Time::now();
        //cells.emplace_back(
        map_->get()->search(in(0), in(1), in(2));//);
        const double time = (cslibs_time::Time::now() - now).nanoseconds();
        access += time;
        access_overall += time;
        ++size;
    }//std::cout << "[OccupancyGridMapper3D]: mean = " << std::to_string(access.getMean()) << std::endl;
    }
    std::cout << "[OccupancyGridMapper3D]: access N | mean | std [ns] = \n"
              << std::to_string(access.getN() / iterations_)
              << " | " << std::to_string(access.getMean())
              << " | " << std::to_string(access.getStandardDeviation())
              << std::endl;
    std::cout << "[OccupancyGridMapper3D]: access mean [ns] = \n"
              << std::to_string(access_overall / size / iterations_)
              << std::endl;
}

const OccupancyGridMapper3D::map_t::ConstPtr OccupancyGridMapper3D::getMap() const
{
    map_->get()->updateInnerOccupancy();
    return map_;
}

bool OccupancyGridMapper3D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    iterations_ = nh.param<double>(param_name("iterations"), 100.0);
    clear_ = nh.param<bool>(param_name("clear"), false);

    const double resolution = nh.param<double>(param_name("resolution"), 1.0);
    map_.reset(new maps::OccupancyGridMap3D(map_frame_, resolution));
    return true;
}

bool OccupancyGridMapper3D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud3d>() ||
           type->isType<cslibs_plugins_data::types::Pointcloud3f>();
}

bool OccupancyGridMapper3D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));
    if (data->isType<cslibs_plugins_data::types::Pointcloud3d>())
        return doProcess<double>(data);
    else
        return doProcess<float>(data);
}

bool OccupancyGridMapper3D::saveMap()
{
    if (!map_) {
        std::cout << "[OccupancyGridMapper3D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[OccupancyGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[OccupancyGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    using path_t = boost::filesystem::path;
    path_t path_root(path_);
    if (!cslibs_ndt::common::serialization::create_directory(path_root))
        return false;

    std::ofstream out((path_root / path_t("stats")).string(), std::fstream::trunc);
    std::string stats_print =
            "[OccupancyGridMapper3D]: N | mean | std | mem = " +
            std::to_string(stats_.getN())
            + " | " + std::to_string(stats_.getMean())
            + " | " + std::to_string(stats_.getStandardDeviation())
            + " | " + std::to_string(map_->get()->memoryUsage()) + "\n";
    std::cout << stats_print << std::endl;
    out << stats_print << std::endl;
    out.close();

    std::string map_path_yaml = (path_ / boost::filesystem::path("map.ot")).string();
    {
        std::ofstream map_out_yaml(map_path_yaml);
        if (!map_out_yaml.is_open()) {
            std::cout << "[OccupancyGridMapper3D '" << name_ << "']: Could not open file '" << map_path_yaml << "'." << std::endl;
            return false;
        }
        if (map_->get()->write(map_out_yaml)) {
            map_out_yaml.close();
            return true;
        }
        else {
            std::cout << "[OccupancyGridMapper3D '" << name_ << "']: Could not write to file '" << map_path_yaml << "'." << std::endl;
            return false;
        }
    }
    return true;
}
}
}
