
#include "cloud_set.hpp"
#include "experiment_viewer.hpp"
int main(int argc, char* argv[]){
    if(argc == 1) return -1;

    PTreePtr conf(new PTree);
    boost::property_tree::read_ini(argv[1], *conf);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(conf->get<std::string>("cloud.hoge"), *cloud);
    CloudSet<pcl::PointXYZ> cloudset(conf, "cloudset");
    ExperimentViewer viewer;
    viewer.set_color_heatmap(0.0);
    viewer.addCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.set_color_heatmap(0.9);
    viewer.addCloud(cloudset);
    viewer.spin();
}
