#include "common.hpp"
#include "utils/random.hpp"
#include "experiment_cloud.hpp"
#include "experiment_viewer.hpp"
/* #include "experiment_iss.hpp" */

int main(int argc, char* argv[]){
    using PT = pcl::PointXYZ; // PT is input Point Type

    shared_ptree pt(new boost::property_tree::ptree);
    boost::property_tree::read_ini(argv[1], *pt);
    ExperimentCloud<PT>::Ptr source(new ExperimentCloud<PT>(pt, "source"));
    ExperimentCloud<PT>::Ptr target(new ExperimentCloud<PT>(pt, "target"));
    source->compute_cloudsize();
    source->compute_normal();
    target->compute_cloudsize();
    target->compute_normal();
    
    /* ExperimentISS<PT>::Ptr iss(new ExperimentISS<PT>(source, pt)); */

    ExperimentViewer exp_viewer("Point Cloud Exp Viewer");

    exp_viewer.set_pointsize(2);
    exp_viewer.set_color_heatmap(0.1);
    exp_viewer.addRawCloud(*source);
    exp_viewer.set_color_heatmap(0.5);
    exp_viewer.addNormalCloud(*source);
    exp_viewer.set_color_heatmap(0.5);
    /* exp_viewer.set_pointsize(4); */
    /* exp_viewer.addNormalCloud(*iss); */
    /* exp_viewer.set_color_heatmap(0.9); */
    /* exp_viewer.addRawCloud(*iss); */

    exp_viewer.setSize(1500,800);
    while ( !exp_viewer.wasStopped() ){
        exp_viewer.spinOnce();
        /* exp_viewer.remove_all_id(); */

        
    }
}
