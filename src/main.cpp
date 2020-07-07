#include "PotentialGrid.h"

#define WINDOW_RADIUS 3.7
#define CONV_ERROR 0.05
#define PUBLISH 1

int main (int argc, char **argv){
    ros::init(argc, argv, "potential");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    //PotentialGrid g(&n, PUBLISH, WINDOW_RADIUS, CONV_ERROR);
    PotentialGrid g(&n);

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