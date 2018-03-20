#ifndef CSLIBS_MAPPING_MAP_H
#define CSLIBS_MAPPING_MAP_H

namespace cslibs_mapping {
namespace maps {
class Map
{
public:
    using Ptr      = std::shared_ptr<Map>;
    using ConstPtr = std::shared_ptr<const Map>;

    template<typename T>
    bool isType() const
    {
        const T *t = dynamic_cast<const T*>(this);
        return t != nullptr;
    }

    template<typename T>
    T const & as() const
    {
        return dynamic_cast<const T&>(*this);
    }
};
}
}

#endif // CSLIBS_MAPPING_MAP_H
