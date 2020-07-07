#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#define OCC_TRESH 90

class MapSub{
    public:
        MapSub(ros::NodeHandle *n){
            sub = n->subscribe("map", 1, &MapSub::mapCallback, this);
            occ_pub = n->advertise<nav_msgs::GridCells>("occ_cells", 1);
            occ_id  = 0;
        }
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
            nav_msgs::GridCells occ_cells;
            occ_cells.cell_height = map->info.resolution;
            occ_cells.cell_width = map->info.resolution;

            int map_size = map->info.width * map->info.height;
            
            for (int i=0; i<map_size; i++)
            {
                if (map->data.at(i) > OCC_TRESH)
                {
                    geometry_msgs::Point p;
                    int y = (int) (i/map->info.width);
                    int x = i%map->info.width;
                    //m.info.origin.position.x
                    p.x = map->info.origin.position.x + (x * map->info.resolution + map->info.resolution/2);
                    p.y = map->info.origin.position.y + (y * map->info.resolution + map->info.resolution/2);
                    p.z = map->info.origin.position.z;

                    ROS_INFO("pushed (%f,%f)",p.x,p.y);
                    occ_cells.cells.push_back(p);
                }
            }
            occ_id ++;
            occ_cells.header.seq = occ_id;
            occ_cells.header.frame_id = "map";
            occ_pub.publish(occ_cells);            
        }
    private:
        ros::Subscriber sub;
        ros::Publisher occ_pub;
        unsigned int occ_id;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "future_potential");
    ros::NodeHandle n;
    
    MapSub s(&n);

    ros::spin();

    return 0; 
}

    