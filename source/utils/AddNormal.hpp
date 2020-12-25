
#ifndef ADD_NORMAL_HPP
#define ADD_NORMAL_HPP

#include <pcl/PCLHeader.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/point_cloud.h>
namespace pcl{

template<typename BasePointType>
struct AddNormal{
    using Type = Normal;
    static_assert(std::is_same<BasePointType, Normal>::value, "AddNormalが定義されていません");
};

template<>
struct AddNormal<PointXYZ>{
    using Type = pcl::PointNormal;
};

template<>
struct AddNormal<PointNormal>{
    using Type = pcl::PointNormal;
};

template<>
struct AddNormal<PointXYZRGB>{
    using Type = pcl::PointXYZRGBNormal;
};

template<>
struct AddNormal<PointXYZRGBNormal>{
    using Type = pcl::PointXYZRGBNormal;
};

template<>
struct AddNormal<PointXYZI>{
    using Type = pcl::PointXYZINormal;
};

template<>
struct AddNormal<PointXYZINormal>{
    using Type = pcl::PointXYZINormal;
};

template<>
struct AddNormal<PointXYZL>{
    using Type = pcl::PointXYZLNormal;
};

template<>
struct AddNormal<PointXYZLNormal>{
    using Type = pcl::PointXYZLNormal;
};

} // namespace pcl

#endif
