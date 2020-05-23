#include "oru_ndt_omfg_grid_mapper_3d.h"

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_time/time.hpp>
#include <cslibs_ndt/serialization/filesystem.hpp>

#include <ndt_map/lazy_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <vector>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::OruNDTOMFGGridMapper3D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
OruNDTOMFGGridMapper3D::~OruNDTOMFGGridMapper3D()
{
    std::string stats_print =
            "[OruNDTOMFGGridMapper3D]: N | mean | std | mem = " +
            std::to_string(stats_.getN())
            + " | " + std::to_string(stats_.getMean())
            + " | " + std::to_string(stats_.getStandardDeviation())
            /*+ " | " + std::to_string(map_->get()->byteSize())*/ + "\n";
    std::cout << stats_print << std::endl;

    perception_oru::SpatialIndex* si = map_->get()->getMyIndex();
    perception_oru::LazyGrid* lg = dynamic_cast<perception_oru::LazyGrid*>(si);
    auto index = [this,&lg](const pcl::PointXYZ& p) -> std::array<int,3> {
        int ix,iy,iz;
        lg->getIndexForPoint(p,ix,iy,iz);
        return {{ix,iy,iz}};
    };
/*
    cslibs_math::statistics::StableDistribution<double,1,6> traversal;
    std::vector<perception_oru::NDTCell*> cells;
    double traversal_overall = 0.0;
    for (int i=0; i<iterations_; ++i) {
        cells.clear();
        cslibs_time::Time now = cslibs_time::Time::now();
        for (typename std::vector<perception_oru::NDTCell*>::iterator it = si->begin(), end = si->end() ; it != end ; ++ it)
            cells.emplace_back(*it);
        const double time = (cslibs_time::Time::now() - now).nanoseconds();
        traversal += time;
        traversal_overall += time;
    }

    std::vector<std::array<int,3>> indices;
    for (typename std::vector<perception_oru::NDTCell*>::iterator it = si->begin(), end = si->end() ; it != end ; ++ it)
        indices.emplace_back(index((*it)->getCenter()));
    std::cout << "[OruNDTOMFGGridMapper3D]: traversal N | mean | std [ms] || traversal normalized mean | std [ns] = \n"
              << std::to_string(traversal.getN())
              << " | " << std::to_string(traversal.getMean() * 1e-6)
              << " | " << std::to_string(traversal.getStandardDeviation() * 1e-6)
              <<" || " << std::to_string(traversal.getMean() / indices.size())
              << " | " << std::to_string(traversal.getStandardDeviation() / indices.size())
              << std::endl;
    std::cout << "[OruNDTOMFGGridMapper3D]: traversal mean [ms / ns] | traversal normalized mean [ns] = \n"
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
    for (typename std::vector<perception_oru::NDTCell*>::iterator it = si->begin(), end = si->end() ; it != end ; ++ it) {
//            indices.emplace_back(index((*it)->getCenter()));
        const std::array<int,3> in = index((*it)->getCenter());

        cslibs_time::Time now = cslibs_time::Time::now();
//        cells.emplace_back(
        map_->get()->getCellAtID(in[0], in[1], in[2]);//);
        const double time = (cslibs_time::Time::now() - now).nanoseconds();
        access += time;
        access_overall += time;
        ++size;
    }std::cout << "[OruNDTOMFGGridMapper3D]: mean = " << std::to_string(access.getMean()) << std::endl;
    }
    std::cout << "[OruNDTOMFGGridMapper3D]: access N | mean | std [ns] = \n"
              << std::to_string(access.getN() / iterations_)
              << " | " << std::to_string(access.getMean())
              << " | " << std::to_string(access.getStandardDeviation())
              << std::endl;
    std::cout << "[OruNDTOMFGGridMapper3D]: access mean [ns] = \n"
              << std::to_string(access_overall / size / iterations_)
              << std::endl;

    std::cout << "[OruNDTOMFGGridMapper3D]: byte_size = " << std::to_string(map_->get()->byteSize()) << std::endl;
}

const OruNDTOMFGGridMapper3D::map_t::ConstPtr OruNDTOMFGGridMapper3D::getMap() const
{
    return map_;
}

bool OruNDTOMFGGridMapper3D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    iterations_ = nh.param<double>(param_name("iterations"), 100.0);
    clear_ = nh.param<bool>(param_name("clear"), false);
    const double resolution = nh.param<double>(param_name("resolution"), 1.0);

    const double ndt_oru_size_x = nh.param<double>(param_name("ndt_oru_size_x"), 0.0);
    const double ndt_oru_size_y = nh.param<double>(param_name("ndt_oru_size_y"), 0.0);
    const double ndt_oru_size_z = nh.param<double>(param_name("ndt_oru_size_z"), 0.0);
    const double ndt_oru_cen_x  = nh.param<double>(param_name("ndt_oru_cen_x"),  0.0);
    const double ndt_oru_cen_y  = nh.param<double>(param_name("ndt_oru_cen_y"),  0.0);
    const double ndt_oru_cen_z  = nh.param<double>(param_name("ndt_oru_cen_z"),  0.0);
    ndt_oru_local_size_x_ = nh.param<double>(param_name("ndt_oru_local_size_x"), ndt_oru_size_x);
    ndt_oru_local_size_y_ = nh.param<double>(param_name("ndt_oru_local_size_y"), ndt_oru_size_y);
    ndt_oru_local_size_z_ = nh.param<double>(param_name("ndt_oru_local_size_z"), ndt_oru_size_z);


    if (ndt_oru_size_x > 1e-3 && ndt_oru_size_y > 1e-3 && ndt_oru_size_z > 1e-3)
        map_.reset(new maps::OruNDTGridMap3D(
                       map_frame_,
                       new perception_oru::LazyGrid(resolution),
                       ndt_oru_cen_x, ndt_oru_cen_y, ndt_oru_cen_z,
                       ndt_oru_size_x, ndt_oru_size_y, ndt_oru_size_z, true));
    else
        map_.reset(new maps::OruNDTGridMap3D(
                       map_frame_,
                       new perception_oru::LazyGrid(resolution)));
    return true;
}

bool OruNDTOMFGGridMapper3D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud3<double>>();
}

bool OruNDTOMFGGridMapper3D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));

    const cslibs_plugins_data::types::Pointcloud3<double> &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud3<double>>();

    cslibs_math_3d::Transform3<double> origin;
    if (tf_->lookupTransform(map_frame_,
                             cloud_data.frame(),
                             ros::Time().fromNSec(cloud_data.timeFrame().start.nanoseconds()),
                             origin,
                             tf_timeout_)) {
        if (const cslibs_math_3d::Pointcloud3d::ConstPtr cloud = cloud_data.points()) {
            pcl::PointCloud<pcl::PointXYZ> pc;
            for (const cslibs_math_3d::Point3d &p : cloud->getPoints()) {
                if (p.isNormal()) {
                    const cslibs_math_3d::Point3d q = origin * p;
                    if (q.isNormal()) {
                        pcl::PointXYZ pt(q(0), q(1), q(2));
                        pc.push_back(pt);
                    }
                }
            }

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(pc, pc, indices);

            const cslibs_time::Time start = cslibs_time::Time::now();
            map_->get()->addPointCloudMeanUpdate(Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), pc,
                                                 Eigen::Vector3d(ndt_oru_local_size_x_, ndt_oru_local_size_y_, ndt_oru_local_size_z_));
            map_->get()->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e9, 255,
                                         Eigen::Vector3d(origin.tx(), origin.ty(), origin.tz()), 0.1);
            const double time = (cslibs_time::Time::now() - start).milliseconds();
            stats_ += time;

            std::cout << "[OruNDTOMFGGridMapper3D]: N = " << stats_.getN() << std::endl;
            return true;
        }
    }
    return false;
}

bool OruNDTOMFGGridMapper3D::saveMap()
{
    if (!map_) {
        std::cout << "[OruNDTOMFGGridMapper3D '" << name_ << "']: No Map." << std::endl;
        return true;
    }

    std::cout << "[OruNDTOMFGGridMapper3D '" << name_ << "']: Saving Map..." << std::endl;
    if (!checkPath()) {
        std::cout << "[OruNDTOMFGGridMapper3D '" << name_ << "']: '" << path_ << "' is not a directory." << std::endl;
        return false;
    }

    using path_t = boost::filesystem::path;
    path_t path_root(path_);
    if (!cslibs_ndt::common::serialization::create_directory(path_root))
        return false;

    std::ofstream out((path_root / path_t("stats")).string(), std::fstream::trunc);
    std::string stats_print =
            "[OruNDTOFGMGridMapper3D]: N | mean | std | mem = " +
            std::to_string(stats_.getN())
            + " | " + std::to_string(stats_.getMean())
            + " | " + std::to_string(stats_.getStandardDeviation())
            + " | " + std::to_string(map_->get()->byteSize()) + "\n";
    std::cout << stats_print << std::endl;
    out << stats_print << std::endl;
    out.close();

    const std::string filename = (path_root / path_t("map.jff")).string();
    if (map_->get()->writeToJFF(filename.c_str())) {
        std::cout << "[OruNDTOMFGGridMapper3D '" << name_ << "']: Saved Map successfully." << std::endl;
        return true;
    }

    return false;
}
}
}
