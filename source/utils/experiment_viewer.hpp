/**
 * @file experiment_viewer.hpp

 * @brief ExperimentCloud visualzer
**/

#ifndef   EXPERIMENT_VIEWER_HPP
#define   EXPERIMENT_VIEWER_HPP
#include "common.hpp"
#include "experiment_cloud.hpp"

#include <pcl/visualization/pcl_visualizer.h>

struct PointType{
    double r, b, g, c;
    int size;
};
void keyboard_callback(const pcl::visualization::KeyboardEvent &event, void* cookie)
{
    PointType* pt;
    pt = (PointType*)cookie;
    if(event.keyDown()){
        switch(event.getKeyCode()){
            case '+':
                pt->size += 1;
                break;
            case '-':
                pt->size -= 1;
                break;
            default:
                break;
        }

    }
}
#define SET_PCL_RENDER_PROP setPointCloudRenderingProperties
class ExperimentViewer : public pcl::visualization::PCLVisualizer{
    public:
    using Ptr = std::shared_ptr<ExperimentViewer>;
    ExperimentViewer(std::string viewer_name = "Experiment Viewer")
        : pt_({1.0, 1.0, 1.0, 0.5, 1})
        , used_id(0)
    {

        setWindowName(viewer_name);
        this->setBackgroundColor(0, 0, 0);
        void* forced_cookie = reinterpret_cast<void*>(&pt_);
        this->registerKeyboardCallback(&keyboard_callback, forced_cookie);
    }
    inline void set_color_heatmap() {
        set_color_heatmap(pt_.c);
    }

    inline void set_color_heatmap(double c) {
        constexpr double pi_ = 4*3.14159265358;
        pt_.c = c;
        if( c < 0){
            set_color_heatmap(c+1.5);
        } else if( c <= 0.25){
            pt_.r = 0.0;
            pt_.g = (-std::cos(pi_*c)+1)/2;
            pt_.b = 1.0;
        }else if( c <= 0.5){
            pt_.r = 0.0;
            pt_.b = (-std::cos(pi_*c)+1)/2;
            pt_.g = 1.0;
        }else if( c <= 0.75){
            pt_.r = (-std::cos(pi_*c)+1)/2;
            pt_.g = 1.0;
            pt_.b = 0.0;
        }else if(c < 1.0) {
            pt_.r = 1.0;
            pt_.g = (-std::cos(pi_*c)+1)/2;
            pt_.b = 0.0;
        }else if(1 < c  && c < 1.25){
            pt_.r = 1.0;
            pt_.g = 0.0;
            pt_.b = (-std::cos(pi_*c)+1)/2;
        }else if(c < 1.50){
            pt_.r = (-std::cos(pi_*c)+1)/2;
            pt_.g = 0.0;
            pt_.b = 1.0;
        }else{
            c*=4;
            c = static_cast<int>(c)%6;
            c/=4;
            set_color_heatmap(c);
        }

    }

    inline void set_color(double r, double g, double b) {
        pt_.r = r; pt_.g = g; pt_.b = b;
    }
    inline void set_size(int width, int height){
	    this->setSize(width, height);
     }
    inline void set_pointsize(int size) {
        pt_.size = size;
    }

    template<typename T>
    void addNormalCloud(
            ExperimentCloud<T> const& cloud, 
            double scale_mr = 10.0,
            int level = 1, 
            int view_point = 0

    ){
        std::string id = cloud.get_id_atr() + "_normalcloud";
        addNormalCloud(cloud, id, scale_mr, level, view_point);
    }

    template<typename T>
    void addNormalCloud(
            ExperimentCloud<T> const& cloud, 
            std::string id, 
            double scale_mr = 10.0,
            int level = 1, 
            int view_point = 0

    ){
        using namespace pcl::visualization;
        auto cld     = cloud.get_cloud_opt();
        auto nrm_opt = cloud.get_normal_opt();
        pcl::PointCloud<pcl::Normal>::ConstPtr nrm(nrm_opt.value());
        auto mr = cloud.get_mr_opt();
        double scale = scale_mr * mr.value();
        this->addPointCloudNormals<T, pcl::Normal>
            (cld.value(), nrm, level, scale, id, view_point);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
        used_id.push_back(id);
    }

    template<typename T>
    void addRawCloud(ExperimentCloud<T>const& cloud)
    {
        using namespace pcl::visualization;
        std::string id = cloud.get_id_atr() + "_rawcloud";
        this->addPointCloud<T>(cloud.get_cloud_opt().value(), id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
        used_id.push_back(id);
    }
    template<typename T>
    void addRawCloud(ExperimentCloud<T>const& cloud, std::string id)
    {
        using namespace pcl::visualization;
        this->addPointCloud<T>(cloud.get_cloud_opt().value(), id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
        used_id.push_back(id);
    }
    void regist_id(std::string id){
        used_id.push_back(id);
    }
    void remove_all_id(){
        for(auto &&id : used_id){
            this->removePointCloud(id);
        }
        used_id.erase(used_id.begin(), used_id.end());
    }
    inline void enable_point_style(std::string id){
        using namespace pcl::visualization;
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }
    private:
    std::vector<std::string> used_id;
    PointType pt_;
    private:
    
};
#undef SET_PCL_RENDER_PROP
#endif    //EXPERIMENT_VIEWER_HPP