#ifndef CSLIBS_MAPPING_MAP_TEMPLATE_HPP
#define CSLIBS_MAPPING_MAP_TEMPLATE_HPP

#include <cslibs_mapping/maps/map.hpp>

namespace cslibs_mapping {
namespace maps {

template <typename T>
class MapTemplate : public Map
{
public:
    using Ptr      = std::shared_ptr<MapTemplate<T>>;
    using ConstPtr = std::shared_ptr<const MapTemplate<T>>;

    using map_t    = T;
    using map_ptr_t= std::shared_ptr<T>;
    template <typename ... args_t>
    inline MapTemplate(const std::string &frame,
                       const args_t &...args) :
        Map(frame),
        map_(new map_t(args...))
    {
    }

    const inline map_ptr_t get() const
    {
        return map_;
    }

private:
    const map_ptr_t map_;
};

}
}

#endif // CSLIBS_MAPPING_MAP_TEMPLATE_HPP
