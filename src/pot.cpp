#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#define OCC_TRESH 70
#define FREE_TRESH 30


enum OccType {UNEXPLORED, OCCUPIED, FREE};

class Cell{
    private:

    public:
        geometry_msgs::Point center;
        OccType occ_type;
        double potential;

        Cell(int val, double x, double y){
            this->center.x = x;
            this->center.y = y;
            this->center.z = 0.0;
            this->potential = -0,01;

            if(val == -1)
                this->occ_type = UNEXPLORED;
            else {
                if (val > OCC_TRESH){
                    this->occ_type = OCCUPIED;
                    this->potential = 1.0;
                }
                else if(val < FREE_TRESH)
                    this->occ_type = FREE;
            }
        }

};

class PotentialGrid
{
    private:
        std::vector<Cell> grid;
        int height_, width_;
        char ready_, should_pub;
        ros::Subscriber map_sub;
        ros::Publisher p_pub;
        unsigned int pf_seq;
    public:
        PotentialGrid(ros::NodeHandle *n, char should_publish){
            ready_ = 0;  
            should_pub = should_publish;
            map_sub = n->subscribe("map", 1, &PotentialGrid::getMap, this);
            if(should_pub){
                ROS_INFO("publishing to /potential_field");
                p_pub = n->advertise<nav_msgs::OccupancyGrid>("potential_field",1);
                pf_seq = 0;
            }
        }

        void getMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
        {
            height_ = map->info.height;
            width_  = map->info.width;

            int map_size = map->info.width * map->info.height;

            nav_msgs::OccupancyGrid pf;

            if(should_pub){
                pf.header.frame_id = "map";
                pf.header.seq = pf_seq++;
                pf.info.height = height_;
                pf.info.width  = width_;
                pf.info.origin = map->info.origin;
                pf.info.resolution = map->info.resolution;
            }

            for (int i=0; i<map_size; i++){
                int y = (int) (i/map->info.width);
                int x = i%map->info.width;
                double center_x = map->info.origin.position.x + (x * map->info.resolution + map->info.resolution/2);
                double center_y = map->info.origin.position.y + (y * map->info.resolution + map->info.resolution/2);

                Cell c (map->data[i], center_x, center_y);
                grid.push_back(c);
                if(should_pub)
                    pf.data.push_back((int)(c.potential * 100));
            }

            if (!ready_)
                ready_ = 1;

            if(should_pub){
                p_pub.publish(pf);
            }
        }

        int ready(){
            return ready_;
        }
        int height (){
            return height_;
        }
        int width (){
            return width_;
        }
        Cell at (int x, int y){
            return grid.at(y*width_ + x);
        }

};

int main (int argc, char **argv){
    ros::init(argc, argv, "potential");
    ros::NodeHandle n;
    int p;
    n.getParam("pub", p);
    PotentialGrid g(&n, p);

    ros::spin();
    return 0;
}