#include "oru_ndt_omfg_grid_mapper_3d.h"

#include <cslibs_plugins_data/types/pointcloud.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <cslibs_time/time.hpp>
#include <cslibs_ndt/serialization/filesystem.hpp>

#include <ndt_map/lazy_grid.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::mapper::OruNDTOMFGGridMapper3D, cslibs_mapping::mapper::Mapper)

namespace cslibs_mapping {
namespace mapper {
const OruNDTOMFGGridMapper3D::map_t::ConstPtr OruNDTOMFGGridMapper3D::getMap() const
{
    return map_;
}

bool OruNDTOMFGGridMapper3D::setupMap(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution = nh.param<double>(param_name("resolution"), 1.0);

    const double ndt_oru_size_x = nh.param<double>(param_name("ndt_oru_size_x"), 0.0);
    const double ndt_oru_size_y = nh.param<double>(param_name("ndt_oru_size_y"), 0.0);
    const double ndt_oru_size_z = nh.param<double>(param_name("ndt_oru_size_z"), 0.0);
    const double ndt_oru_cen_x  = nh.param<double>(param_name("ndt_oru_cen_x"),  0.0);
    const double ndt_oru_cen_y  = nh.param<double>(param_name("ndt_oru_cen_y"),  0.0);
    const double ndt_oru_cen_z  = nh.param<double>(param_name("ndt_oru_cen_z"),  0.0);
    ndt_oru_local_size_x_ = nh.param<double>(param_name("ndt_oru_local_size_x"), 0.0);
    ndt_oru_local_size_y_ = nh.param<double>(param_name("ndt_oru_local_size_y"), 0.0);
    ndt_oru_local_size_z_ = nh.param<double>(param_name("ndt_oru_local_size_z"), 0.0);

    if (ndt_oru_size_x > 1e-3 && ndt_oru_size_y > 1e-3 && ndt_oru_size_z > 1e-3)
        map_.reset(new maps::OruNDTGridMap3D(
                       map_frame_,
                       new lslgeneric::LazyGrid(resolution),
                       ndt_oru_cen_x, ndt_oru_cen_y, ndt_oru_cen_z,
                       ndt_oru_size_x, ndt_oru_size_y, ndt_oru_size_z, true));
    else
        map_.reset(new maps::OruNDTGridMap3D(
                       map_frame_,
                       new lslgeneric::LazyGrid(resolution)));
    return true;
}

bool OruNDTOMFGGridMapper3D::uses(const data_t::ConstPtr &type)
{
    return type->isType<cslibs_plugins_data::types::Pointcloud>();
}

void OruNDTOMFGGridMapper3D::process(const data_t::ConstPtr &data)
{
    assert (uses(data));

    const cslibs_plugins_data::types::Pointcloud &cloud_data = data->as<cslibs_plugins_data::types::Pointcloud>();

    tf::Transform o_T_d_tmp;
    if (tf_->lookupTransform(map_frame_,
                             cloud_data.getFrame(),
                             ros::Time(cloud_data.getTimeFrame().start.seconds()),
                             o_T_d_tmp,
                             tf_timeout_)) {
        cslibs_math_3d::Transform3d origin = cslibs_math_ros::tf::conversion_3d::from(o_T_d_tmp);
        if (const cslibs_math_3d::Pointcloud3d::Ptr cloud = cloud_data.getPoints()) {
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
            stats_print_ += "[OruNDTOMFGGridMapper3D]: N | current | mean | std | mem = " +
                    std::to_string(stats_.getN()) + " | " + std::to_string(time)
                    + " | " + std::to_string(stats_.getMean())
                    + " | " + std::to_string(stats_.getStandardDeviation())
                    + " | " + std::to_string(map_->get()->byteSize()) + "\n";
        }
    }
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
    out << stats_print_ << std::endl;
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
