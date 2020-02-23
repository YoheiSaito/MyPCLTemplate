#ifndef ISS_HPP
#define ISS_HPP
#include <pcl/keypoints/iss_3d.h>

template <typename PointInT,typename PointOutT,typename NormalT = pcl::Normal>
class MyISSKeypoint : public pcl::ISSKeypoint3D<PointInT, PointOutT, NormalT>
{
    public:
    using Ptr      = 
       boost::shared_ptr<      MyISSKeypoint<PointInT, PointOutT, NormalT> >;
    using ConstPtr = 
       boost::shared_ptr<const MyISSKeypoint<PointInT, PointOutT, NormalT> >;
    using PointCloudIn  =
        typename pcl::Keypoint<PointInT, PointOutT>::PointCloudIn;
    using PointCloudOut =
        typename pcl::Keypoint<PointInT, PointOutT>::PointCloudOut;

    using PointCloudN = pcl::PointCloud<NormalT>;
    using PointCloudNPtr = typename PointCloudN::Ptr;
    using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

    using OctreeSearchIn = pcl::octree::OctreePointCloudSearch<PointInT>;
    using OctreeSearchInPtr = typename OctreeSearchIn::Ptr;

    using pcl::Keypoint<PointInT, PointOutT>::input_;
    using pcl::Keypoint<PointInT, PointOutT>::tree_;
    using pcl::Keypoint<PointInT, PointOutT>::search_radius_;
    MyISSKeypoint(double salient_radius = 0.0001)
        : pcl::ISSKeypoint3D<PointInT, PointOutT, NormalT>(salient_radius)
    {
    }

    std::vector<int> compute_non_maximum(double r_nms){
        double n = input_->size();
        bool* feat_max = new bool[n];
        auto&& eigen = this->third_eigen_value_;
        for(int i = 0; i < n; i++) feat_max[i] = true;
        for(int i = 0; i < n; i++){
            if(!feat_max[i])
                continue;
            const PointInT& p = (input_->points[i]);
            if ((eigen[i] <= 0.0) || (!pcl::isFinite(p)))
                continue;
            std::vector<int> indx;
            std::vector<float> dist;
            this->search_method_(i, r_nms, indx, dist);
            const int n_neighbors = static_cast<int> (indx.size());
            if (n_neighbors >= this->min_neighbors_) {
                bool is_max = true;
                for (int j = 0 ; j < n_neighbors; j++){
                    if (eigen[i] < eigen[indx[j]]){
                        is_max = false;
                    }else if(eigen[i] > eigen[indx[j]]){
                        feat_max[indx[j]] = false;
                    }
                }
                if(is_max){
                    feat_max[i] = true;
                }
            }
        }
        std::vector<int> nms;
        for(int i = 0; i < n; i++){
            if(feat_max[i]){
                nms.push_back(i);
            }
        }
        delete[] feat_max;
        return nms;
    }
    void compute_and_restrict(pcl::PointCloud<PointOutT>& out, int num = 10){
        this->compute(out);
        int n_max = out.size();
        if( num < 0 || n_max  == num) return;
        double r_max = this->non_max_radius_;
        double r_min = 0;
        int n_min = 2*num - n_max;
        int prev_n = -1;
        while(r_max != r_min){
            double r = (r_max + r_min)/2;
            std::vector<int> indx = compute_non_maximum(r);
            auto n = indx.size();
            /* std::cout << indx.size() << '\t'<< r << std::endl; */
            if( n == prev_n ){
                r_max = static_cast<double>(prev_n)/num*r_max;
                continue;
            }
            prev_n = n;
            if( n == num ){
                pcl::copyPointCloud(*input_, indx, out);
                break;
            }else if( n < num ){
                r_max = r;
            }else{
                r_min = r;
            }
        }
        
    }
};


#endif
