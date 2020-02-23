/**
 * @file experiment_viewer.hpp

 * @brief CloudSet visualzer
**/

#ifndef   EXPERIMENT_VIEWER_HPP
#define   EXPERIMENT_VIEWER_HPP

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#if __has_include("cloud_set.h")
#include "cloud_set.h"
#endif

struct PointStyle {
    double r, b, g, c;
    int size;
};
inline void 
keyboard_callback(const pcl::visualization::KeyboardEvent &event, void* cookie)
{
    PointStyle* pt;
    pt = (PointStyle*)cookie;
    if(event.keyDown()){
        switch(event.getKeyCode()){
            case '+':
                pt->size += 1;
                break;
            case '-':
                pt->size -= 1;
                break;
            case '!':
                pt->c -= 0.01;
                break;
            case '@':
                pt->c += 0.01;
                break;
            default:
                break;
        }

    }
}
#define SET_PCL_RENDER_PROP setPointCloudRenderingProperties
class ExperimentViewer : public pcl::visualization::PCLVisualizer{
    public:
    PointStyle pt_;
    using Ptr = std::shared_ptr<ExperimentViewer>;
    ExperimentViewer(std::string viewer_name = "Experiment Viewer")
        : pt_({1.0, 1.0, 1.0, 0.5, 1})
    {

        setWindowName(viewer_name);
        this->setSize(1200, 800);
        this->setBackgroundColor(1, 1, 1);
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
        }else if(c <= 1.0) {
            pt_.r = 1.0;
            pt_.g = (-std::cos(pi_*c)+1)/2;
            pt_.b = 0.0;
        }else if(c <= 1.25){
            pt_.r = 1.0;
            pt_.g = 0.0;
            pt_.b = (-std::cos(pi_*c)+1)/2;
        }else if(c <= 1.50){
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

#ifdef CLOUD_SET_H
    template<typename T>
    inline void addNormal(
            CloudSet<T>& cloud, 
            double scale_mr = 4.0,
            int level = 1, 
            int view_point = 0

    ){
        addNormal(cloud, cloud.atr_, scale_mr, level, view_point);
    }

    template<typename T>
    inline void addNormal(
            CloudSet<T>& cloud, 
            std::string id, 
            double scale_mr = 4.0,
            int level = 1, 
            int view_point = 0

    ){
        using namespace pcl::visualization;
        auto mr = cloud.mr_;
        double scale = scale_mr * mr;
        this->addPointCloudNormals<T, pcl::Normal>
            (cloud.cloud_, cloud.normal_, level, scale, id, view_point);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR, pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }
#endif
    template<typename T>
    inline void 
    addCloud (typename pcl::PointCloud<T>::Ptr const& cloud, std::string id)
    {
        using namespace pcl::visualization;
        this->addPointCloud<T>(cloud, id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }


#ifdef CLOUD_SET_H
    template<typename T>
    inline void addCloud(CloudSet<T>const& cloud)
    {
        using namespace pcl::visualization;
        this->addCloud(cloud, cloud.atr_);
    }
    template<typename T>
    inline void addCloud(CloudSet<T>const& cloud, std::string id)
    {
        using namespace pcl::visualization;
        this->addPointCloud<T>(cloud.cloud_, id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }
    template<typename T>
    inline void addNormalWithTransform(
            CloudSet<T>const& cloud,
            Eigen::Affine3f& t,
            double scale = 4.0,
            int level = 1, 
            int view_point = 0
    ) {
        using namespace pcl::visualization;
        std::string id = cloud.atr_+"transform_with_normal";
        addNormalWithTransform(cloud, id, t, scale, level, view_point);
    }

    template<typename T>
    inline void addNormalWithTransform
    (
            CloudSet<T>const& cloud,
            std::string id, 
            Eigen::Affine3f& t,
            double scale = 4.0,
            int level = 1, 
            int view_point = 0
    ) {
        using namespace pcl::visualization;
        pcl::PointCloud<pcl::PointNormal> base;
        pcl::PointCloud<pcl::PointNormal>::Ptr 
            points(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields( *cloud.cloud_, *cloud.normal_, base);
        pcl::transformPointCloudWithNormals( base, *points, t);
        double scale_mr = scale * cloud.mr_;
        this->addPointCloudNormals<pcl::PointNormal>
            (points, level, scale_mr, id, view_point);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }
#endif
    template<typename T>
    inline void  addCloudWithTransform (
            typename pcl::PointCloud<T>::Ptr const& cloud, 
            Eigen::Affine3f& t,
            std::string id
    )
    {
        using namespace pcl::visualization;
        typename pcl::PointCloud<T>::Ptr p(new typename pcl::PointCloud<T>);
        pcl::transformPointCloud(*cloud, *p, t);
        this->addPointCloud<T>(p, id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }

    template<typename T>
    inline void addCloudWithTransform (
            typename pcl::PointCloud<T> const& cloud,
            Eigen::Affine3f& t,
            std::string id
    ) {
        using namespace pcl::visualization;
        typename pcl::PointCloud<T>::Ptr p(new typename pcl::PointCloud<T>);
        pcl::transformPointCloud(cloud, *p, t);
        this->addPointCloud<T>(p, id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
     }

#ifdef CLOUD_SET_H
    template<typename T>
    inline void
    addCloudWithTransform(CloudSet<T>const& cloud, Eigen::Affine3f& t)
    {
        std::string id = "transformed"+cloud.atr_;
        this->addCloudWithTransform(cloud, id, t);
    }
    template<typename T>
    inline void addCloudWithTransform
    (CloudSet<T>const& cloud, std::string id, Eigen::Affine3f& t)
    {
        using namespace pcl::visualization;
        typename pcl::PointCloud<T>::Ptr points(new pcl::PointCloud<T>);
        pcl::transformPointCloud( *cloud.cloud_, *points, t);
        this->addPointCloud<T>(points, id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }
#endif
    void config_render_id(std::string id){
        using namespace pcl::visualization;
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_COLOR,pt_.r,pt_.g,pt_.b,id);
        this->SET_PCL_RENDER_PROP(PCL_VISUALIZER_POINT_SIZE, pt_.size, id);
    }
    void screenshot(std::string filename){
        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = 
            vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInput(this->getRenderWindow());
        /* windowToImageFilter->SetInputBufferTypeToRGBA(); */
        windowToImageFilter->Update();

        vtkSmartPointer<vtkPNGWriter> writer = 
            vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(filename.c_str());
        writer->SetInputConnection(windowToImageFilter->GetOutputPort());
        writer->Write();

    }
   
};
extern ExperimentViewer::Ptr exp_viewer;
#undef SET_PCL_RENDER_PROP
#endif    //EXPERIMENT_VIEWER_HPP
