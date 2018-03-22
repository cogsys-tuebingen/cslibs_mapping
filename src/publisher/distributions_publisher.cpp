#include "distributions_publisher.h"

#include <cslibs_ndt_3d/DistributionArray.h>

#include <cslibs_mapping/maps/ndt_grid_map_3d.h>
#include <cslibs_mapping/maps/occupancy_ndt_grid_map_3d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_mapping::publisher::DistributionsPublisher, cslibs_mapping::publisher::Publisher)

namespace cslibs_mapping {
namespace publisher {
bool DistributionsPublisher::uses(const map_t::ConstPtr &map) const
{
    return map->isType<cslibs_mapping::maps::NDTGridMap3D>() ||
           map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D>();
}

void DistributionsPublisher::doAdvertise(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    const std::string topic = nh.param<std::string>(param_name("topic"), "/cslibs_mapping/" + name_);

    const bool occupancy_ndt = nh.param<bool>(param_name("occupancy_ndt"), false);
    if (occupancy_ndt) {
        const double prob_prior    = nh.param(param_name("prob_prior"),    0.5);
        const double prob_free     = nh.param(param_name("prob_free"),     0.45);
        const double prob_occupied = nh.param(param_name("prob_occupied"), 0.65);
        ivm_.reset(new cslibs_gridmaps::utility::InverseModel(
                       prob_prior, prob_free, prob_occupied));
    }

    publisher_ = nh.advertise<cslibs_ndt_3d::DistributionArray>(topic, 1);
}

void DistributionsPublisher::publish(const map_t::ConstPtr &map, const ros::Time &time)
{
    if (map->isType<cslibs_mapping::maps::NDTGridMap3D>())
        publishNDTGridMap3D(map, time);
    else if (map->isType<cslibs_mapping::maps::OccupancyNDTGridMap3D>())
        publishOccupancyNDTGridMap3D(map, time);
    else
        std::cout << "[DistributionsPublisher]: Got wrong map type!" << std::endl;
}

void DistributionsPublisher::publishNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
{
    using local_map_t = cslibs_ndt_3d::dynamic_maps::Gridmap;
    auto sample = [] (const local_map_t::distribution_t *d,
                      const local_map_t::point_t &p) -> double {
        return d ? d->data().sampleNonNormalized(p) : 0.0;
    };
    auto sample_bundle = [&sample] (const local_map_t::distribution_bundle_t *b,
                                    const local_map_t::point_t &p) -> double {
        return 0.125 * (sample(b->at(0), p) +
                        sample(b->at(1), p) +
                        sample(b->at(2), p) +
                        sample(b->at(3), p) +
                        sample(b->at(4), p) +
                        sample(b->at(5), p) +
                        sample(b->at(6), p) +
                        sample(b->at(7), p));
    };

    const local_map_t::Ptr &m = map->as<cslibs_mapping::maps::NDTGridMap3D>().getMap();
    std::vector<std::array<int, 3>> bundle_indices;
    m->getBundleIndices(bundle_indices);

    cslibs_ndt_3d::DistributionArray distributions;
    distributions.header.stamp    = time;
    distributions.header.frame_id = map->getFrame();
    for (auto &bi : bundle_indices) {
        if (const local_map_t::distribution_bundle_t *b = m->getDistributionBundle(bi)) {
            local_map_t::distribution_t::distribution_t d;
            for (std::size_t i = 0 ; i < 8 ; ++ i)
                d += b->at(i)->getHandle()->data();

            if (d.getN() == 0)
                continue;
            const auto &mean         = d.getMean();
            const auto &eigenvalues  = d.getEigenValues();
            const auto &eigenvectors = d.getEigenVectors();
            const auto &covariance   = d.getCovariance();

            cslibs_ndt_3d::Distribution distr;
            distr.id.data = b->id();
            for (int i = 0; i < 3; ++ i) {
                distr.mean[i].data          = mean(i);
                distr.eigen_values[i].data  = eigenvalues(i);
            }
            for (int i = 0; i < 9; ++ i) {
                distr.eigen_vectors[i].data = eigenvectors(i);
                distr.covariance[i].data    = covariance(i);
            }
            distr.prob.data = sample_bundle(b, local_map_t::point_t(mean));

            distributions.data.emplace_back(distr);
        }
    }

    publisher_.publish(distributions);
}

void DistributionsPublisher::publishOccupancyNDTGridMap3D(const map_t::ConstPtr &map, const ros::Time &time)
{
    using local_map_t = cslibs_ndt_3d::dynamic_maps::OccupancyGridmap;
    auto sample = [this] (const local_map_t::distribution_t *d,
                          const local_map_t::point_t &p) -> double {
        return (d && d->getDistribution()) ?
                    (d->getDistribution()->sampleNonNormalized(p) * d->getOccupancy(ivm_)) :
                    0.0;
        };
    auto sample_bundle = [&sample] (const local_map_t::distribution_bundle_t *b,
                                    const local_map_t::point_t &p) -> double {
        return 0.125 * (sample(b->at(0), p) +
                        sample(b->at(1), p) +
                        sample(b->at(2), p) +
                        sample(b->at(3), p) +
                        sample(b->at(4), p) +
                        sample(b->at(5), p) +
                        sample(b->at(6), p) +
                        sample(b->at(7), p));
    };

    const local_map_t::Ptr &m = map->as<cslibs_mapping::maps::OccupancyNDTGridMap3D>().getMap();
    std::vector<std::array<int, 3>> bundle_indices;
    m->getBundleIndices(bundle_indices);

    cslibs_ndt_3d::DistributionArray distributions;
    distributions.header.stamp    = time;
    distributions.header.frame_id = map->getFrame();
    for (auto &bi : bundle_indices) {
        if (const local_map_t::distribution_bundle_t *b = m->getDistributionBundle(bi)) {
            local_map_t::distribution_t::distribution_t d;
            for (std::size_t i = 0 ; i < 8 ; ++ i)
                if (b->at(i)->getDistribution())
                    d += *(b->at(i)->getDistribution());

            if (d.getN() == 0)
                continue;
            const auto &mean         = d.getMean();
            const auto &eigenvalues  = d.getEigenValues();
            const auto &eigenvectors = d.getEigenVectors();
            const auto &covariance   = d.getCovariance();

            cslibs_ndt_3d::Distribution distr;
            distr.id.data = b->id();
            for (int i = 0; i < 3; ++ i) {
                distr.mean[i].data          = mean(i);
                distr.eigen_values[i].data  = eigenvalues(i);
            }
            for (int i = 0; i < 9; ++ i) {
                distr.eigen_vectors[i].data = eigenvectors(i);
                distr.covariance[i].data    = covariance(i);
            }
            distr.prob.data = sample_bundle(b, local_map_t::point_t(mean));

            distributions.data.emplace_back(distr);
        }
    }

    publisher_.publish(distributions);
}
}
}
