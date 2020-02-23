#ifndef EXPERIMENT_BOUNDARY_HPP
#define EXPERIMENT_BOUNDARY_HPP

#include "../cloud_set.h"
#include <pcl/features/boundary.h>
template<typename PointT>
void 
estimatie_boundary(
        CloudSet<PointT>& input,
        std::vector<int>& boundary,
        std::vector<int>& not_boundary,
        double radius = 4,
        double thickness = 4,
        double angle = 90
){

    // boundary estimation
    using NT = pcl::Normal;
    typename pcl::BoundaryEstimation<PointT, NT, pcl::Boundary> be;
    be.setSearchMethod(input.kdtree_);
    be.setRadiusSearch(radius* input.mr_);
    be.setAngleThreshold( 3.1415926535*(angle)/180.0 );
    be.setInputCloud  (input.cloud_);
    be.setInputNormals(input.normal_);
    pcl::PointCloud<pcl::Boundary> tmp_bnd;
    be.compute(tmp_bnd);
    
    // extruct points around boundary
    boundary.reserve(2*input.cloud_->size());
    not_boundary.reserve(2*input.cloud_->size());
    thickness = thickness*input.mr_;
    for(int i = 0; i < input.cloud_->points.size(); i++){
        if(tmp_bnd[i].boundary_point){
            std::vector<int> idx;
            std::vector<float> sq_dst;
            auto&& p = input.cloud_->points[i];
            input.kdtree_->radiusSearch(p, thickness, idx, sq_dst);
            for(auto&& j : idx){
                boundary.push_back(j);
            }
        }
    }
    // filter common points
    std::sort(boundary.begin(), boundary.end());
    auto end_itr = std::unique(boundary.begin(), boundary.end());
    boundary.erase(end_itr, boundary.end());

    // extract points that not in around boundary
    not_boundary.resize(0);
    not_boundary.reserve(input.cloud_->size() - boundary.size());
    std::vector<int> all(input.cloud_->size());
    std::iota(all.begin(), all.end(), 0);
    std::set_difference(
            all.begin(), all.end(), 
            boundary.begin(), boundary.end(),
            std::inserter(not_boundary, not_boundary.end())
    );
}

#endif
