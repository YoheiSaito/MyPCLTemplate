#ifndef     COMMON_HPP
#define     COMMON_HPP
#include <memory>
#include <vector>
#include <string>
#include <optional>
#include <initializer_list>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
using shared_ptree = std::shared_ptr<boost::property_tree::ptree>;

#endif    //COMMON_HPP
