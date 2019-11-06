#include "common.hpp"
#include "experiment_cloud.hpp"
#include "experiment_viewer.hpp"

int main(int argc, char* argv[]){
    shared_ptree pt(new boost::property_tree::ptree);
    boost::property_tree::read_ini(argv[1], *pt);
    ExperimentCloud<pcl::PointXYZ> source(pt, "source");
    source.compute_normal();
    std::cout << source.bbdd_ << std::endl;
    std::cout << source.mr_   << std::endl;
    std::cout << source.center_.transpose()   << std::endl;

    pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

    viewer.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>
        ( source.cloud_, source.cloud_, 1, 4*source.mr_, "normal");
    /* viewer.addPointCloud<pcl::PointNormal>(source.cloud_); */
    while ( !viewer.wasStopped() ){
        viewer.spinOnce();
    }
}
