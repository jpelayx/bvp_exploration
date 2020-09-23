#include "PotentialGrid.h"

#define WINDOW_RADIUS 3.7
#define CONV_ERROR 0.05
#define PUBLISH 1

int main (int argc, char **argv){
    ros::init(argc, argv, "potential");
    ros::NodeHandle n;

    ros::Rate loop_rate(15);

    std::string ns = ros::this_node::getNamespace();

    PotentialGrid g(&n, ns);

    while(ros::ok())
    {
        ros::spinOnce();
        if(g.initialized){
           g.followPotential();            
        }
        loop_rate.sleep();
    }
    return 0;
}