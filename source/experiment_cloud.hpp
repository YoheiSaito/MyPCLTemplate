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
    using NormalCloud    = typename pcl::PointCloud<pcl::Normal>;
    using PointCloudPtr  = typename PointCloud::Ptr;
    using NormalCloudPtr = typename NormalCloud::Ptr;

    // type alias for kdtree
    using Kdtree    = typename pcl::search::KdTree<PointT>;
    using KdtreePtr = typename Kdtree::Ptr;
    ExperimentCloud(shared_ptree ini_ptr, std::string atr)
        : ini_(ini_ptr)
        , atr_(atr)
        , computed_size_(false), computed_normal_(false)
        , cloud_  (new PointCloud()),  normal_ (new NormalCloud())
        , kdtree_ (new Kdtree ())
    {
        load_cloud();
        kdtree_->setInputCloud(cloud_);
    }

    void load_cloud(){
        common_dir_ = ini_->get<std::string>("common.dataset_path");
        boost::filesystem::path filename(common_dir_);
        filename /= ini_->get<std::string>( atr_ + ".filename");
        pcl::io::loadPCDFile(filename.c_str(), *cloud_);
        remove_nan_from_cloud();
    }

    protected:
    ExperimentCloud(void) {}
    // property 
    shared_ptree ini_;
    std::string common_dir_;
    bool computed_normal_, computed_size_;

    // point cloud
    PointCloudPtr cloud_;
    NormalCloudPtr normal_;
    KdtreePtr kdtree_;

    // attribute of point cloud
    double mr_;
    double bbdd_;
    Eigen::Vector3f center_;
    Eigen::Vector3f size_;
    Eigen::Vector3f min_;
    Eigen::Vector3f max_;

    // member method
    std::string atr_;
    public:
    void compute_cloudsize(){
        const int K(2);
        using namespace boost::accumulators;
        accumulator_set<float, stats<tag::max, tag::min> > acc_x, acc_y, acc_z;
        accumulator_set<float, stats<tag::median> > acc_range;
        std::vector<int> idx(2);
        std::vector<float> sq_dist(2);
        for(auto&& p : cloud_->points){
            kdtree_->nearestKSearch( p, K, idx, sq_dist);
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

    NormalCloudPtr compute_normal(double normal_r = -1){
        if( computed_normal_ && normal_r == -1 )
            return normal_;
        if( !computed_size_){
            compute_cloudsize();
        }
        auto r_opt = ini_->get_optional<double>(atr_ + ".normal_radius_mr");
        double r = 10 * mr_;
        if(normal_r != -1)
            r = normal_r * mr_;
        else if(r_opt)
            r =  r_opt.get() * mr_;

        typename pcl::NormalEstimation<PointT, pcl::Normal>  ne;
        ne.setInputCloud(cloud_);
        ne.setRadiusSearch (r);
        ne.compute(*normal_);
        remove_nan_from_normal();
        computed_normal_ = true;
        return normal_;
    }

    //getter 
    // get is not const method
    // get_opt is const method 
    NormalCloudPtr get_normal(){
        if( !computed_normal_ )
            compute_normal();
        return normal_;
    }
    PointCloudPtr get_cloud() const {
        return cloud_;
    }
    KdtreePtr get_kdtree() const{
        return kdtree_;
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
    Eigen::Vector3f get_normal_size(){
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

    std::optional<NormalCloudPtr> get_normal_opt() const {
        if( !computed_normal_)
            return std::nullopt;
        return normal_;
    }
    
    std::optional<PointCloudPtr> get_cloud_opt() const {
        return cloud_;
    }

    std::optional<KdtreePtr> get_kdtree_opt() const{
        return kdtree_;
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
    std::optional<Eigen::Vector3f> get_normal_size_opt() const {
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
    std::optional<std::string> get_id_atr_opt() const{
        return atr_;
    }
    std::string get_id_atr() const{
        return atr_;
    }
    protected:
    void remove_nan_from_cloud(void){
        typename pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud_);
        pass.filter(*cloud_);
    }
    void remove_nan_from_normal(void){
        // remove nan
        if( !computed_normal_ )
            return;
        pcl::PointCloud<pcl::PointNormal>::Ptr tmp
            (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud_, *normal_, *tmp);
        pcl::PassThrough<pcl::PointNormal> pass;
        pass.setFilterFieldName ("normal_x");
        pass.setFilterLimits(-1.001, 1.001);
        pass.setInputCloud(tmp);
        pass.setFilterFieldName ("normal_y");
        pass.setFilterLimits(-1.001, 1.001);
        pass.setInputCloud(tmp);
        pass.setFilterFieldName ("normal_z");
        pass.setFilterLimits(-1.001, 1.001);
        pass.setInputCloud(tmp);
        pass.filter(*tmp);

        cloud_  = PointCloudPtr (new PointCloud);
        normal_ = NormalCloudPtr(new NormalCloud);
        cloud_ ->resize(tmp->size());
        normal_->resize(tmp->size());
        for(int i = 0; i < tmp->points.size(); i++){
            cloud_->points[i].getVector4fMap() 
                    = tmp->points[i].getVector4fMap();
            normal_->points[i].getNormalVector4fMap() 
                    = tmp->points[i].getNormalVector4fMap();
        }
    }
};


#endif    //EXPERIMENT_CLOUD_HPP
