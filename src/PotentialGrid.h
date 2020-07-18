#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <mutex>

#define OCC_TRESH 70
#define FREE_TRESH 30

enum OccType {UNEXPLORED, OCCUPIED, MARKED_OCCUPIED, FREE};
enum FrontType {FRONTIER, MARKED_FRONTIER, NDA};

class Cell
{
    public:
        double potential;
        OccType occupation;
        FrontType frontier;

        Cell(int);
        void update(int);
        int show();
};

class PotentialGrid
{
    private:
        ros::Subscriber map_sub;
        ros::Publisher  potential_pub;
        ros::Publisher vector_pub;
        ros::Publisher path_pub;
        ros::Publisher vector_field_pub;
        ros::Publisher vel_pub;
        ros::Publisher ponto;

        nav_msgs::Path path;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener* tf_listener;

        std::vector<std::vector<Cell*> > grid;
        
        int current_grid_x, current_grid_y;
        geometry_msgs::Point map0;

        int param_pub_pot,
            param_pub_gradient_vec,
            param_pub_path;
        double param_window_radius,
               param_potential_convergence_tol;

        std::mutex mtx;
    
    public:
        PotentialGrid(ros::NodeHandle*);
        void getMap(const nav_msgs::OccupancyGrid::ConstPtr&);

        void updatePotential(int,int,int,int);
        bool setGoal(int,int,int,int);
        bool isFrontier(int,int);
        bool nearOccupied(int,int);

        int current_position(geometry_msgs::TransformStamped*);
        int grid_x(geometry_msgs::TransformStamped);
        int grid_y(geometry_msgs::TransformStamped);

        std::vector<double> normalizedGradient(int,int);
        
        double normalizeAngle(double);

        void followPotential();

        void publishPotentialField(nav_msgs::MapMetaData);
        void publishVector(std::vector<double>, geometry_msgs::TransformStamped);
        void publishPath(geometry_msgs::TransformStamped); 
        void publishVectorField();

        int width, height;
        double resolution;
        bool initialized;

};