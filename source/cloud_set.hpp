#ifndef CLOUD_SET_HPP
#define CLOUD_SET_HPP
#include "cloud_set.h"

template<typename PointType>
CloudSet<PointType>&
CloudSet<PointType>::appends(CloudSet<PointType> const& base)
{
    for(auto&& p : base. cloud_->points)
        this-> cloud_->push_back(p);
    for(auto&& p : base.normal_->points)
        this-> normal_->push_back(p);
    for(auto&& p : base.lrf_x_->points)
        this-> lrf_x_->push_back(p);
    for(auto&& p : base.lrf_y_->points)
        this-> lrf_y_->push_back(p);
    for(auto&& p : base.lrf_z_->points)
        this-> lrf_z_->push_back(p);

    kdtree_->setInputCloud(cloud_);
    compute_size();
    compute_normal();
    
    return *this;
}

template<typename PointType>
void CloudSet<PointType>::load_cloud() {
    using Path = boost::filesystem::path;
    Path filename(ptree_->get<std::string>(  "common.dataroot"));
    filename /=   ptree_->get<std::string>( atr_ + ".filename") ;
    std::string ext(filename.extension().c_str());
    if(ext == "ply")
        pcl::io::loadPLYFile(filename.c_str(), *cloud_);
    else 
        pcl::io::loadPCDFile(filename.c_str(), *cloud_);

    
    pcl::VoxelGrid<PointType> grid;
    auto lopt = ptree_->get_optional<double>( atr_ + ".voxelsize");
    if(lopt){
        double leaf = lopt.get();
        grid.setInputCloud(cloud_);
        grid.setLeafSize(leaf, leaf, leaf);
        grid.setMinimumPointsNumberPerVoxel(1);
        grid.filter(*cloud_);
    }

    std::vector<int> _corrs;
    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, _corrs);
    kdtree_->setInputCloud(cloud_);
}

template<typename PointType>
void CloudSet<PointType>::set_truth(){
    boost::system::error_code error;
    using Path = boost::filesystem::path;
    Path fname(ptree_->get<std::string>(  "common.dataroot"));
    fname /=   ptree_->get<std::string>( atr_ + ".filename") ;
    std::string fullpath(fname.c_str());
    auto dir_last = fullpath.find_last_of('/');
    auto file_last = fullpath.find_last_of('.');
    const std::string atr_name(fullpath, dir_last+1, file_last-dir_last-1);
    const std::string dir_name(fullpath, 0, dir_last);
    const std::string file_name(dir_name + "/TRUTH.ini");
    if( boost::filesystem::exists(file_name, error) )
        load_truth_transform (file_name, atr_name);
}

template<typename PointType>
void 
CloudSet<PointType>::load_truth_transform(std::string fname, std::string atr){
    PTree pt;
    boost::property_tree::read_ini(fname, pt);
    auto trans_opt = pt.get_optional<std::string>(atr + ".line");
    if(!trans_opt)
        return;
    std::string repository = pt.get<std::string>(atr + ".repository");
    if(repository=="stanford")
    {
        std::vector<double> T(7);
        std::stringstream ss;
        ss << trans_opt.get();
        for(auto&& t : T){
            std::string tmp;
            ss >> tmp;
            t = std::stod(tmp);
        }
        Eigen::Translation3f t = Eigen::Translation3f(T[0], T[1], T[2]);
        Eigen::Quaternionf R = Eigen::Quaternionf (-T[6], T[3], T[4], T[5]);
        Eigen::Affine3f tmp;
        tmp = (t*R);
        truth_transform_ = tmp;
    }
    else if(repository=="blensor")
    {
        std::vector<double> T(6);
        std::stringstream ss;
        ss << trans_opt.get();
        for(auto&& t : T){
            std::string tmp;
            ss >> tmp;
            t = std::stod(tmp);
        }
        constexpr double deg2rad = 3.141592653589/180.0;
        Eigen::Affine3f tmp;
        tmp = pcl::getTransformation(
                T[0], T[1], T[2], 
                deg2rad*T[3], 
                deg2rad*T[4], 
                deg2rad*T[5]
        );
        truth_transform_ = tmp;
    }else{
        std::cerr 
            << "Repository \"" 
            << repository 
            << "\" is not defined" 
            << std::endl;
        truth_transform_ = Eigen::Affine3f::Identity();
    }
}

template<typename PointType>
void CloudSet<PointType>::compute_size(bool force_compute){
    if(!force_compute && computed_size_)
        return;

    const int K(2);
    using namespace boost::accumulators;
    kdtree_->setInputCloud(cloud_);
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

template<typename PointType>
void CloudSet<PointType>::compute_normal(double normal_r){
    // set radius
    if(!computed_size_)
        compute_size();
    auto r_opt = ptree_->get_optional<double>(atr_ + ".normal_radius_mr");
    double r = 10 * mr_;
    if(normal_r != -1)
        r = normal_r * mr_;
    else if(r_opt)
        r =  r_opt.get() * mr_;
    // set normal
    typename pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(cloud_);
    ne.setRadiusSearch (r);
    ne.compute(*normal_);
    std::vector<int> corrs;
    pcl::removeNaNNormalsFromPointCloud(*normal_, *normal_, corrs);
    pcl::copyPointCloud(*cloud_, corrs, *cloud_);
    // copy cloud
    kdtree_->setInputCloud(cloud_);
    computed_normal_ = true;
}

template<typename PointType>
double CloudSet<PointType>::compute_mse(
        pcl::PointCloud<PointType> & base, double p
){
    int n = base.points.size();
    int np = p*n;
    if(np < 0 || np >= n)
        np = n;
    std::vector<float> sq_dists(n);
    for(int i = 0; i < n; i++){
        std::vector<int> indx;
        std::vector<float> sq_dist;
        auto&& p = base.points[i];
        this->kdtree_->nearestKSearch(p, 1, indx, sq_dist);
        sq_dists[i] = sq_dist[0];
    }
    std::sort(sq_dists.begin(), sq_dists.end());
    double sum = 1e-10;
    for(int i = 0; i < np; i++){
        sum += sq_dists[i];
    }
    return sum/(np+1e-30);
}
template<typename PointType>
double CloudSet<PointType>::compute_mse(CloudSet<PointType> & base, double p)
{
    int n = base.cloud_->points.size();
    int np = p*n;
    if(np < 0 || np >= n)
        np = n;
    std::vector<float> sq_dists(n);
    for(int i = 0; i < n; i++){
        std::vector<int> indx;
        std::vector<float> sq_dist;
        auto&& p = base.cloud_->points[i];
        this->kdtree_->nearestKSearch(p, 1, indx, sq_dist);
        sq_dists[i] = sq_dist[0];
    }
    std::sort(sq_dists.begin(), sq_dists.end());
    double sum = 1e-10;
    for(int i = 0; i < np; i++){
        sum += sq_dists[i];
    }
    return sum/(np+1e-30);
}

template<typename PointType>
void CloudSet<PointType>::copy_etc(CloudSet<PointType> & base){
    this->computed_size_   = base.computed_size_;
    this->mr_              = base.mr_;
    this->bbdd_            = base.bbdd_;
    this->center_          = base.center_;
    this->min_             = base.min_;
    this->max_             = base.max_;
    this->size_            = base.size_;
    this->computed_normal_ = base.computed_size_;;
}

#endif
