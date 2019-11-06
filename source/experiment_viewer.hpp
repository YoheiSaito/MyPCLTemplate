/**
 * @file experiment_viewer.hpp

 * @brief ExperimentCloud visualzer
**/

#ifndef   EXPERIMENT_VIEWER_HPP
#define   EXPERIMENT_VIEWER_HPP
#include "common.hpp"
#include "experiment_cloud.hpp"

#define SET_PCL_RENDER_PROP setPointCloudRenderingProperties
class ExperimentViewer{
    public:
    using Ptr = std::shared_ptr<ExperimentViewer>;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    ExperimentViewer(std::string viewer_name = "Experiment Viewer")
        : viewer(new pcl::visualization::PCLVisualizer(viewer_name))
        , r_(1.0), b_(1.0), g_(1.0), size_(1.0)
    {
        viewer->setBackgroundColor(0, 0, 0);
    }
    pcl::visualization::PCLVisualizer::Ptr operator() (){
        return viewer;
    }
    pcl::visualization::PCLVisualizer& operator=(ExperimentViewer v){
        viewer = v.viewer;
        return *viewer;
    }
    inline void set_color(double r, double g, double b) {
        r_ = r; g_ = g; b_ = b;
    }
    inline void set_size(int width, int height){
	    viewer->setSize(width, height);
     }
    inline void set_pointsize(int size) {
        size_ = size;
    }
    template<typename T>
    void addNormalCloud(
            ExperimentCloud<T>& cloud, 
            std::string id, 
            double scale_mr = 10.0,
            int level = 1, 
            int view_point = 0

    ){
        using namespace pcl::visualization;
        auto cld = cloud.get_cloud_opt();
        auto mr = cloud.get_mr_opt();
        double scale = scale_mr * mr.value();
        viewer->addPointCloudNormals<pcl::PointNormal>
            (cld.value(), level, scale, id, view_point);
        viewer->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR, r_, g_, b_, id);
        viewer->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, size_, id);
    }

    template<typename T>
    void addRawCloud(ExperimentCloud<T>const& cloud, std::string id)
    {
        using namespace pcl::visualization;
        viewer->addPointCloud<T>(cloud.get_rawcloud_opt().value(), id);
        viewer->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR, r_, g_, b_, id);
        viewer->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, size_, id);
    }
    private:
    std::string iterative_id;
    double r_, g_, b_;
    double size_;
};
#undef SET_PCL_RENDER_PROP

#endif    //EXPERIMENT_VIEWER_HPP
