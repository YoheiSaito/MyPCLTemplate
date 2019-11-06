#ifndef  EXPERIMENT_CLOUD_HPP
#define  EXPERIMENT_CLOUD_HPP

#include "common.hpp"
#include <pcl/io/pcd_io.h>
template<typename PointT>
class ExperimentCloud {
    public:
    // type alias for point cloud
    using PointCloud     = typename pcl::PointCloud<PointT>;
    using NormalCloud    = typename pcl::PointCloud<pcl::PointNormal>;
    using PointCloudPtr  = typename PointCloud::Ptr;
    using NormalCloudPtr = typename NormalCloud::Ptr;

    // type alias for kdtree
    using RawKdtree    = typename pcl::search::KdTree<PointT>;
    using Kdtree       = typename pcl::search::KdTree<pcl::PointNormal>;
    using RawKdtreePtr = typename RawKdtree::Ptr;
    using KdtreePtr    = typename Kdtree::Ptr;

    ExperimentCloud(shared_ptree ini_ptr, std::string atr)
        : ini_(ini_ptr)
        , atr_(atr)
        , computed_size_(false), computed_normal_(false)
        , raw_cloud_  (new PointCloud()),  cloud_ (new NormalCloud())
        , raw_kdtree_ (new RawKdtree ()), kdtree_ (new      Kdtree())
    {
        ini_ = ini_ptr;
        common_dir_ = ini_->get<std::string>("common.dataset_path");
        boost::filesystem::path filename(common_dir_);
        filename /= ini_->get<std::string>( atr_ + ".filename");
        pcl::io::loadPCDFile(filename.c_str(), *raw_cloud_);
        raw_kdtree_->setInputCloud(raw_cloud_);
    }

    protected:
    // property 
    shared_ptree ini_;
    std::string atr_;
    std::string common_dir_;
    bool computed_normal_, computed_size_;

    // point cloud
    public:
    PointCloudPtr raw_cloud_;
    NormalCloudPtr cloud_;
    KdtreePtr kdtree_;
    RawKdtreePtr raw_kdtree_;

    // attribute of point cloud
    double mr_;
    double bbdd_;
    Eigen::Vector3f center_;
    Eigen::Vector3f size_;
    Eigen::Vector3f min_;
    Eigen::Vector3f max_;

    // member method
    public:
    void compute_cloudsize(){
        const int K(2);
        using namespace boost::accumulators;
        accumulator_set<float, stats<tag::max, tag::min> > acc_x, acc_y, acc_z;
        accumulator_set<float, stats<tag::median> > acc_range;
        std::vector<int> idx(2);
        std::vector<float> sq_dist(2);
        for(auto&& p : raw_cloud_->points){
            raw_kdtree_->nearestKSearch( p, K, idx, sq_dist);
            acc_range(sq_dist[1]);
            acc_x(p.x);
            acc_y(p.y);
            acc_z(p.z);
        }
        mr_ = sqrt(extract::median(acc_range));
        max_[0] =  extract::max(acc_x);
        min_[0] =  extract::min(acc_x);
        max_[1] =  extract::max(acc_y);
        min_[1] =  extract::min(acc_y);
        max_[2] =  extract::max(acc_z);
        min_[2] =  extract::min(acc_z);
        size_   = max_ - min_;
        bbdd_   = size_.norm();
        center_ = (max_ + min_)/2.0;
        computed_size_ = true;
    }

    NormalCloudPtr get_cloud(){
        if( !computed_normal_ )
            compute_normal();
        return cloud_;
    }
    NormalCloudPtr compute_normal(){
        if(computed_normal_)
            return cloud_;
        if( !computed_size_){
            compute_cloudsize();
        }
        copyPointCloud(*raw_cloud_, *cloud_);
        double r = ini_->get<double>(atr_ + ".normal_radius_mr") * mr_;
        typename pcl::NormalEstimation<PointT, pcl::PointNormal>  ne;
        ne.setInputCloud(raw_cloud_);
        ne.setRadiusSearch (r);
        ne.compute(*cloud_);
        kdtree_->setInputCloud(cloud_);
        computed_normal_ = true;
        return cloud_;
    }
};

#endif    //EXPERIMENT_CLOUD_HPP
