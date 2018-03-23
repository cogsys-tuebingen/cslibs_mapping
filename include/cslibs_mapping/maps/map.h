#ifndef CSLIBS_MAPPING_MAP_H
#define CSLIBS_MAPPING_MAP_H

#include <eigen3/Eigen/Eigen>

namespace cslibs_mapping {
namespace maps {
class Map
{
public:
    using Ptr      = std::shared_ptr<Map>;
    using ConstPtr = std::shared_ptr<const Map>;

    inline Map() = delete;
    inline Map(const std::string &frame) :
        frame_(frame)
    {
    }
    virtual inline ~Map() = default;

    template <typename T>
    bool isType() const
    {
        const T *t = dynamic_cast<const T*>(this);
        return t != nullptr;
    }

    template <typename T>
    T const & as() const
    {
        return dynamic_cast<const T&>(*this);
    }

    inline std::string getFrame() const
    {
        return frame_;
    }

private:
    std::string frame_;
};
}
}

#endif // CSLIBS_MAPPING_MAP_H
