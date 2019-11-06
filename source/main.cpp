#include "common.hpp"
#include "experiment_cloud.hpp"
#include "experiment_viewer.hpp"

int main(int argc, char* argv[]){
    shared_ptree pt(new boost::property_tree::ptree);
    boost::property_tree::read_ini(argv[1], *pt);
    ExperimentCloud<pcl::PointXYZ> source(pt, "source");
    source.compute_cloudsize();
    source.compute_normal();
    ExperimentViewer exp_viewer("Point Cloud Exp Viewer");
    exp_viewer.set_color_heatmap(0.5);
    while ( !exp_viewer.wasStopped() ){
        exp_viewer.set_color_heatmap();
        exp_viewer.addRawCloud( source, "fuga");
        exp_viewer.addNormalCloud( source, "hoge", 5, 1);
        exp_viewer.spinOnce();
        exp_viewer.removePointCloud("hoge");
        exp_viewer.removePointCloud("fuga");
    }
}
