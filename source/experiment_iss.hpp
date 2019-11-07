#ifndef EXPERIMENT_ISS_HPP
#define EXPERIMENT_ISS_HPP
#include "common.hpp"
#include "experiment_cloud.hpp"

#include <pcl/keypoints/iss_3d.h>

template<typename PointT>
class ExperimentISS: public ExperimentCloud<PointT>{

    public:
    using ExpCloud    = ExperimentCloud<PointT>;
    using ExpCloudPtr = typename ExperimentCloud<PointT>::Ptr;

    using Ptr            = typename std::shared_ptr<ExperimentISS<PointT>>;
    using PointCloud     = typename pcl::PointCloud<PointT>;
    using NormalCloud    = typename pcl::PointCloud<pcl::PointNormal>;
    using PointCloudPtr  = typename PointCloud::Ptr;
    using NormalCloudPtr = typename NormalCloud::Ptr;

    // type alias for kdtree
    using RawKdtree    = typename pcl::search::KdTree<PointT>;
    using Kdtree       = typename pcl::search::KdTree<pcl::PointNormal>;
    using RawKdtreePtr = typename RawKdtree::Ptr;
    using KdtreePtr    = typename Kdtree::Ptr;

    ExperimentISS(ExpCloudPtr p, shared_ptree ini, std::string atr = "iss")
        : ExperimentCloud<PointT>()
    {
        process_atr_ = atr;
        this->ini_ = (ini);
        this->atr_ = p->get_id_atr() + atr;
        this->cloud_      = NormalCloudPtr(new NormalCloud());
        this->kdtree_     = KdtreePtr(new Kdtree());
        this->raw_cloud_  = PointCloudPtr(new PointCloud()); 
        this->raw_kdtree_ = RawKdtreePtr(new RawKdtree ()); 

        this->computed_size_   = (false);
        this->computed_normal_ = (true);
        
        p->compute_normal();
        get_parameters();        

        pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
        iss.setSearchMethod(p->get_raw_kdtree());
        iss.setInputCloud  (p->get_rawcloud());
        iss.setSalientRadius(r_salient_ * p->get_mr());
        iss.setNonMaxRadius (r_nms_     * p->get_mr());
        iss.setThreshold21(thresh21_);
        iss.setThreshold32(thresh32_);
        iss.setMinNeighbors(min_neighbors_);
        iss.setNumberOfThreads(threads_);
        iss.compute(*(this->raw_cloud_));

        // copy the nearest normal vector
        std::vector<int> idx(1);
        std::vector<float> sq_dist(1);
        const int N = this->raw_cloud_->points.size();
        std::vector<int> indeces(N);
        auto kd = p->get_cloud_kdtree();
        auto base = p->get_cloud();
        for(size_t i = 0; i < N; i++){
            pcl::PointNormal query;
            query.x = this->raw_cloud_->points[i].x;
            query.y = this->raw_cloud_->points[i].y;
            query.z = this->raw_cloud_->points[i].z;
            query.normal_x = 0; query.normal_y = 0; query.normal_z = 0;
            kd->nearestKSearch(query, 1, idx, sq_dist);
            indeces[i] = idx[0];
        } 
        pcl::copyPointCloud<pcl::PointNormal>( *base, indeces, *(this->cloud_));
        // update kdtree
        this->raw_kdtree_->setInputCloud(this->raw_cloud_);
        this->kdtree_->setInputCloud(this->cloud_);

        this->compute_cloudsize();
        this->mr_ = p->get_mr();
    }
    private:
    std::string process_atr_;
    double thresh21_, thresh32_;
    double r_salient_, r_nms_;
    int min_neighbors_, threads_;
    void get_parameters(){
        std::string atr = process_atr_;
        shared_ptree ini = this->ini_;
        auto t21        = ini->get_optional<double>(atr + ".thresh21");
        auto t32        = ini->get_optional<double>(atr + ".thresh32");
        auto o_salient_ = ini->get_optional<double>(atr + ".r_salient");
        auto o_nms      = ini->get_optional<double>(atr + ".r_nms");
        auto o_min      = ini->get_optional<int>(atr + ".min_neighbors");
        auto o_thread   = ini->get_optional<int>(atr + ".threads");
        thresh21_      = t21        ? t21.get()        : 0.975; 
        thresh32_      = t32        ? t32.get()        : 0.975; 
        r_salient_     = o_salient_ ? o_salient_.get() :  10.0;
        r_nms_         = o_nms      ? o_nms.get()      :   8.0;
        min_neighbors_ = o_min      ? o_min.get()      :     5;
        o_thread       = o_thread   ? o_thread.get()   :     1;
    }
};

#endif
