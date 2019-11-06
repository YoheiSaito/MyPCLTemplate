/**
 * @file experiment_cloud.hpp

 * @brief This is the a collection of cloud operation
**/

#ifndef  EXPERIMENT_CLOUD_HPP
#define  EXPERIMENT_CLOUD_HPP

#include "common.hpp"
template<typename PointT>
class ExperimentCloud {
    public:
    // type alias for point cloud
    using Ptr            = typename std::shared_ptr<ExperimentCloud<PointT>>;
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

        // remove nan
        typename pcl::PassThrough<PointT> pass;
        pass.setInputCloud(raw_cloud_);
        pass.filter(*raw_cloud_);

        raw_kdtree_->setInputCloud(raw_cloud_);
    }

    protected:
    // property 
    shared_ptree ini_;
    std::string atr_;
    std::string common_dir_;
    bool computed_normal_, computed_size_;

    // point cloud
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
        // remove nan
        pcl::PassThrough<pcl::PointNormal> pass;
        pass.setFilterFieldName ("normal_x");
        pass.setFilterLimits(-1.1, 1.1);
        pass.setInputCloud(cloud_);
        pass.filter(*cloud_);
        kdtree_->setInputCloud(cloud_);
        computed_normal_ = true;
        return cloud_;
    }

    //getter 
    // get is not const method
    // get_opt is const method 
    NormalCloudPtr get_cloud(){
        if( !computed_normal_ )
            compute_normal();
        return cloud_;
    }
    KdtreePtr get_cloud_kdtree(){
        if( !computed_normal_ )
            compute_normal();
        return kdtree_;
    }
    PointCloudPtr get_rawcloud() const {
        return raw_cloud_;
    }
    RawKdtreePtr get_raw_kdtree() const{
        return raw_kdtree_;
    }
    double get_mr(){
        if( !computed_size_ )
            compute_cloudsize();
        return mr_;
    }
    double get_bbdd(){
        if( !computed_size_ )
            compute_cloudsize();
        return bbdd_;
    }
    Eigen::Vector3f get_center(){
        if( !computed_size_ )
            compute_cloudsize();
        return center_;
    }
    Eigen::Vector3f get_cloud_size(){
        if( !computed_size_ )
            compute_cloudsize();
        return size_;
    }
    Eigen::Vector3f get_max_point(){
        if( !computed_size_ )
            compute_cloudsize();
        return max_;
    }
    Eigen::Vector3f get_min_point(){
        if( !computed_size_ )
            compute_cloudsize();
        return min_;
    }

    std::optional<NormalCloudPtr> get_cloud_opt() const {
        if( !computed_normal_)
            return std::nullopt;
        return cloud_;
    }
    
    std::optional<PointCloudPtr> get_rawcloud_opt() const {
        return raw_cloud_;
    }

    std::optional<KdtreePtr> get_cloud_kdtree_opt() const{
        if( !computed_normal_ )
            return std::nullopt;
        return kdtree_;
    }
    std::optional<RawKdtreePtr> get_raw_kdtree_opt() const{
        return raw_kdtree_;
    }
    std::optional<double> get_mr_opt() const {
        if( !computed_size_ )
            return std::nullopt;
        return mr_;
    }
    std::optional<double> get_bbdd_opt() const {
        if( !computed_size_ )
            return std::nullopt;
        return bbdd_;
    }
    std::optional<Eigen::Vector3f> get_center_opt() const {
        if( !computed_size_ )
            return std::nullopt;
        return center_;
    }
    std::optional<Eigen::Vector3f> get_cloud_size_opt() const {
        if( !computed_size_ )
            return std::nullopt;
        return size_;
    }
    std::optional<Eigen::Vector3f> get_max_point_opt() const {
        if( !computed_size_ )
            return std::nullopt;
        return max_;
    }
    std::optional<Eigen::Vector3f> get_min_point_opt() const {
        if( !computed_size_ )
            return std::nullopt;
        return min_;
    }
};


#endif    //EXPERIMENT_CLOUD_HPP
