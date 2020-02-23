#ifndef CLOUD_SET_H
#define CLOUD_SET_H

#include <array>
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

using PTree    = boost::property_tree::ptree;
using PTreePtr = boost::shared_ptr<PTree>;
template<typename PointType>
struct CloudSet{
    using Ptr    = typename boost::shared_ptr<CloudSet<PointType>>;
    using Cloud  = typename pcl::PointCloud<PointType>;
    using Normal = typename pcl::PointCloud<pcl::Normal>;
    using KdTree = typename pcl::search::KdTree<PointType>;

    using CloudPtr  = typename Cloud::Ptr;
    using NormalPtr = typename Normal::Ptr;
    using KdTreePtr = typename KdTree::Ptr;
    std::string  atr_;
    PTreePtr   ptree_;
    CloudPtr   cloud_;
    NormalPtr normal_;
    KdTreePtr kdtree_;
    NormalPtr lrf_x_, lrf_y_, lrf_z_;
    Eigen::Affine3f truth_transform_;

    // cloud features
    double mr_, bbdd_;
    Eigen::Vector3f center_, min_, max_, size_;
    bool computed_size_;
    // normal 
    bool computed_normal_;

    CloudSet(PTreePtr ini, std::string atr)
        : ptree_(ini)
        , atr_(atr)
        ,  cloud_(new Cloud)
        , normal_(new Normal)
        , kdtree_(new KdTree)
        , lrf_x_(new Normal)
        , lrf_y_(new Normal)
        , lrf_z_(new Normal)
        , computed_size_(false)
        , computed_normal_(false)
    {
        load_cloud();
        set_truth();
    }
    CloudSet( CloudSet<PointType>& base, pcl::PointCloud<PointType>& seed, 
            double r = 0.4)
        : ptree_(base.ptree_)
        , atr_(base.atr_)
        ,  cloud_(new Cloud)
        , normal_(new Normal)
        , kdtree_(new KdTree)
        , lrf_x_(new Normal)
        , lrf_y_(new Normal)
        , lrf_z_(new Normal)
        , computed_size_(base.computed_size_)
        , computed_normal_(base.computed_size_)

    {
        atr_ = "near_extracted_"+atr_;
        copy_etc(base);
        std::vector<int> seed_indx;
        r *= this->mr_;
        for(auto&& p : seed.points){
            std::vector<int>   indx;
            std::vector<float> sq_dist;
            kdtree_->radiusSearch(p, r, indx, sq_dist);
            for(auto&& i : indx){
                seed_indx.push_back(i);
            }
        }
        std::sort(seed_indx.begin(), seed_indx.end());
        auto end = std::unique(seed_indx.begin(), seed_indx.end());
        seed_indx.erase(end, seed_indx.end());
        pcl::copyPointCloud(*base.normal_, seed_indx, *this->normal_);
        pcl::copyPointCloud(*base. cloud_, seed_indx, *this-> cloud_);
        if(lrf_x_->size() == base.normal_->size()){
            pcl::copyPointCloud(*base.lrf_x_, seed_indx, *this->lrf_x_);
            pcl::copyPointCloud(*base.lrf_y_, seed_indx, *this->lrf_y_);
            pcl::copyPointCloud(*base.lrf_z_, seed_indx, *this->lrf_z_);
        }

        kdtree_->setInputCloud(cloud_);
    }
    CloudSet(CloudSet<PointType> & base, Eigen::Affine3f& initial_trans)
        : ptree_(base.ptree_)
        , atr_(base.atr_)
        ,  cloud_(new Cloud)
        , normal_(new Normal)
        , kdtree_(new KdTree)
        , lrf_x_(new Normal)
        , lrf_y_(new Normal)
        , lrf_z_(new Normal)
        , computed_size_(base.computed_size_)
        , computed_normal_(base.computed_size_)
    {
        atr_ = "transformed_"+atr_;
        copy_etc(base);
        this->normal_->resize(base.normal_->size());
        Eigen::Matrix3f rot;
        rot = initial_trans.rotation().matrix();
        for(int i = 0; i < base.normal_->size(); i++){
            this->normal_->points[i].getNormalVector3fMap() 
                = rot * base.normal_->points[i].getNormalVector3fMap();
        }
        this-> lrf_x_->resize(base. lrf_x_->size());
        this-> lrf_y_->resize(base. lrf_y_->size());
        this-> lrf_z_->resize(base. lrf_z_->size());
        for(int i = 0; i < base.lrf_x_->size(); i++){
            this-> lrf_x_->points[i].getNormalVector3fMap() 
                = rot * base. lrf_x_->points[i].getNormalVector3fMap();
            this-> lrf_y_->points[i].getNormalVector3fMap() 
                = rot * base. lrf_y_->points[i].getNormalVector3fMap();
            this-> lrf_z_->points[i].getNormalVector3fMap() 
                = rot * base. lrf_z_->points[i].getNormalVector3fMap();
        }
        pcl::transformPointCloud(*base.cloud_, *this->cloud_, initial_trans);
        kdtree_->setInputCloud(cloud_);
    }
    inline void copy_etc(CloudSet<PointType> & base);
    inline void load_cloud(); 
    inline void set_truth();
    inline void load_truth_transform(std::string fname, std::string atr);
    inline void compute_size(bool forced_compute = false);
    inline void compute_normal(double mr = -1);
    inline double compute_mse(CloudSet<PointType> & base, double p=-1);
    inline double compute_mse(pcl::PointCloud<PointType> &, double p=-1);
    inline CloudSet<PointType>& appends(CloudSet<PointType> const &);
};
#include "cloud_set.hpp"
#endif
