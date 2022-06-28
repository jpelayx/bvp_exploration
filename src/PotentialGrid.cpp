#include "PotentialGrid.h"
#include <tf2/utils.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>
#include <queue>

PotentialGrid::PotentialGrid(ros::NodeHandle *n, std::string name_space)
{
    initialized = 0;
    n->getParam("pub_potential", param_pub_pot);
    n->getParam("pub_gradient", param_pub_gradient_vec);
    n->getParam("pub_path", param_pub_path);
    n->getParam("window_radius", param_window_radius);
    n->getParam("conv_tol", param_potential_convergence_tol);

    std::cout << "namespace: " << name_space << std::endl;
    ns="";
     if(name_space != "/"){
        ns = name_space+"/";
        // ns.erase(0,1);
    }
    std::cout << "ns: " << ns << std::endl;
    map_sub = n->subscribe(ns+"map", 1, &PotentialGrid::getMap, this);
    // map_sub = n->subscribe("/map",1,&PotentialGrid::getMap, this);
    tf_listener = new tf2_ros::TransformListener (tf_buffer);
    
    vel_pub = n->advertise<geometry_msgs::Twist>("cmd_vel",1);

    if(param_pub_pot){
        ROS_INFO("publishing to /potential_field");
        potential_pub = n->advertise<nav_msgs::OccupancyGrid>(ns+"potential_field",1);
    }
    if(param_pub_gradient_vec){
        ROS_INFO("publishing to /gradient");
        vector_pub = n->advertise<visualization_msgs::Marker>(ns+"gradient",1);
    }
    if(param_pub_path){
        path_pub = n->advertise<nav_msgs::Path>(ns+"path",1);
        path.header.frame_id = "map";
        path.header.seq = 0;
    }
    vector_field_pub = n->advertise<visualization_msgs::Marker>(ns+"vector_field", 1);
    front_pub = n->advertise<nav_msgs::GridCells>(ns+"frontiers", 1);
}

void PotentialGrid::getMap(const nav_msgs::OccupancyGrid::ConstPtr& map){
    mtx.lock();
    height = map->info.height;
    width  = map->info.width;
    resolution = map->info.resolution;
    map0 = map->info.origin.position;
    mtx.unlock();

    int map_size = height * width;
    int x0=0, xf=height, y0=0, yf=width;

    ROS_INFO("updating info");

    geometry_msgs::TransformStamped current_pos;
    if(current_position(&current_pos) == -1){
        return;
    }    
    // ROS_INFO("current position = %f, %f\n height, width, resolution = %d, %d, %f", current_pos.transform.translation.x, current_pos.transform.translation.y, height, width, resolution);
    // ROS_INFO("position on grid = %d, %d", current_grid_x, current_grid_y);

    mtx.lock();
    geometry_msgs::Twist vel;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.linear.x = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    vel_pub.publish(vel);

    if(!initialized){
        //new grid
        // ROS_INFO("new grid");
        grid.clear();
        grid.resize(width);

        for(int i=0; i<width; i++){
            for(int j=0; j<height; j++){
                Cell *c = new Cell (map->data[i+j*width]);
                grid[i].push_back(c);
            }
        }
        initialized = true;
    }
    else{
        //update activation window
        ROS_INFO("reusing cells");
        
        x0 = grid_x(current_pos) - (int)(param_window_radius/map->info.resolution);
        if(x0<0)
            x0=0;
        xf = grid_x(current_pos) + (int)(param_window_radius/map->info.resolution);
        if(xf>width)
            xf=width;
        y0 = grid_y(current_pos) - (int)(param_window_radius/map->info.resolution);
        if(y0<0)
            y0=0;
        yf = grid_y(current_pos) + (int)(param_window_radius/map->info.resolution);
        if(yf>height)
            yf=height;

        for(int i=x0; i<xf; i++){
            for(int j=y0; j<yf; j++){
                grid[i][j]->update(map->data[i+j*width]);
            }
        }
    }
    ROS_INFO("updating potential (%d to %d, %d to %d", x0, xf, y0, yf);
    updatePotential(x0, xf, y0, yf);
    publishVectorField();
    mtx.unlock();

    // std::vector<float> grad_descent = normalizedGradient(current_grid_x,current_grid_y);
    // ROS_INFO("grad vec (%f,%f)", grad_descent[0], grad_descent[1]);
   
    if(param_pub_pot){
        publishPotentialField(map->info);
    }
}

int PotentialGrid::current_position(geometry_msgs::TransformStamped *pos){
    geometry_msgs::TransformStamped current_pos;
    try{
            std::string name = ns;
            name.erase(0,1);
            current_pos = tf_buffer.lookupTransform(name+"map", name+"base_link", ros::Time(0));
            // current_pos = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
            *pos = current_pos;
            return 0;
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return -1;
    }
}

int PotentialGrid::grid_x(geometry_msgs::TransformStamped pos){
    return (int) ((pos.transform.translation.x - map0.x)/resolution);
}

int PotentialGrid::grid_y(geometry_msgs::TransformStamped pos){
    return (int) ((pos.transform.translation.y - map0.y)/resolution);
}

double PotentialGrid::worldX(int x){
    return (x * resolution) + map0.x + resolution/2;
}

double PotentialGrid::worldY(int y){
    return (y * resolution) + map0.y + resolution/2;
}

void PotentialGrid::updatePotential(int x, int x_max, int y, int y_max){
    expandObstacles(x,x_max,y,y_max);
    if(!setGoal(x,x_max,y,y_max)){
        x = 0;
        x_max = width;
        y = 0;
        y_max = height;
        expandObstacles(x,x_max,y,y_max);
        setGoal(x, x_max, y, y_max);
        //fazer: comportamento caso nao ache fronteira
    }
    double error = 100, old;
    int iterations = 0;

    while(error > param_potential_convergence_tol){
        error = 0;
        for(int i=x; i<x_max; i++){
            for (int j=y; j<y_max; j++){
                if (grid.at(i).at(j)->occupation == FREE){
                    old = grid.at(i).at(j)->potential;
                    grid.at(i).at(j)->potential = (grid.at(i-1).at(j)->potential +
                                                   grid.at(i+1).at(j)->potential +
                                                   grid.at(i).at(j-1)->potential +
                                                   grid.at(i).at(j+1)->potential)/4;
                    error += pow(old - grid.at(i).at(j)->potential, 2);
                }
            }
        }
        iterations++;
    }
    ROS_INFO("potential converged with %d iterations", iterations);
}

bool PotentialGrid::setGoal(int x, int x_max, int y, int y_max){
    bool frontier_found = false;
    ROS_INFO("setting up frontiers");
    for(int i=x; i<x_max; i++){
        for(int j=y; j<y_max; j++){
            if(isFrontier(i,j)){
                if(!frontier_found)
                    frontier_found = true;
                grid[i][j]->frontier  = FRONTIER;
            }     
        }
    }
    if(!frontier_found)
        return false;
    ROS_INFO("frontiers found");
    
    std::vector<int> frontier_centers;
    
    for(int i=x; i<x_max; i++)
        for(int j=x; j<y_max; j++)
            if(grid[i][j]->frontier == FRONTIER){
                std::vector<int> frontier;
                geometry_msgs::Point center;
                center.x = 0.0;                 
                center.y = 0.0; 
                center.z = 0.0; 
                grid[i][j]->frontier = MARKED_FRONTIER;

                int size = 0;
                std::queue<int> q;
                q.push(i);
                q.push(j);

                while(!q.empty()){
                    size ++;
                    int x_front = q.front();
                    q.pop();
                    int y_front = q.front();
                    q.pop();

                    center.x += x_front; 
                    center.y += y_front;                     
                    frontier.push_back(x_front);
                    frontier.push_back(y_front);

                    for(int x_adj=x_front-1; x_adj<= x_front+1; x_adj++)
                        for(int y_adj=y_front-1; y_adj<=y_front+1; y_adj++)
                            if(grid[x_adj][y_adj]->frontier == FRONTIER){
                                grid[x_adj][y_adj]->frontier = MARKED_FRONTIER;
                                q.push(x_adj);
                                q.push(y_adj);
                            }
                        
                }

                center.x = center.x/size;
                center.y = center.y/size;

                float min_dist = FLT_MAX;
                int closest_x, closest_y;
                for(int n=0; n<frontier.size(); n+=2){
                    float dist = sqrt(pow(center.x-frontier[n], 2) + pow(center.y-frontier[n+1], 2));
                    if(dist < min_dist){
                        closest_x = frontier[n];
                        closest_y = frontier[n+1];
                        min_dist = dist;
                    }
                }
                frontier_centers.push_back(closest_x);
                frontier_centers.push_back(closest_y);
            }
    for(int n=0; n<frontier_centers.size(); n+=2){
        ROS_INFO("center: %d, %d == %f, %f", frontier_centers[n], frontier_centers[n+1], worldX(frontier_centers[n]), worldY(frontier_centers[n+1]));
        grid[frontier_centers[n]][frontier_centers[n+1]]->potential = 0.0;
    }
    publishFrontiers(frontier_centers);
    return true;     
}
        
void PotentialGrid::expandObstacles(int x, int x_max, int y, int y_max){
    int rad = 3;
    for(int i=x; i<x_max; i++)
        for(int j=y; j<y_max; j++)
        {
            // if(nearOccupied(i,j)){
            //     grid[i][j]->occupation = OCCUPIED_EXP;
            //     grid[i][j]->potential  = 1.0;
            // }
            if(grid[i][j]->occupation == OCCUPIED){
                for(int inear = i-rad; inear<=i+rad; inear++)
                    for(int jnear=j-rad; jnear<=j+rad; jnear++){
                        if(grid[inear][jnear]->occupation == FREE){
                            grid[inear][jnear]->occupation = OCCUPIED_EXP;
                            grid[inear][jnear]->potential  = 1.0;
                        }
                    }
            }
        }
}

bool PotentialGrid::isFrontier(int i, int j){
    if(grid[i][j]->occupation != UNEXPLORED)
        return false;
    if(i>0)
    if(grid[i-1][j]->occupation == FREE)
        return true;
    if(i<width-1)
        if(grid[i+1][j]->occupation == FREE)
            return true;
    if(j>0)
        if(grid[i][j-1]->occupation == FREE)
            return true;
    if(j<height-1)
        if(grid[i][j+1]->occupation == FREE)
            return true;
    return false;
}

bool PotentialGrid::nearOccupied(int i, int j){
    int rad = 3;
    if(grid[i][j]->occupation != FREE)
        return false;
    for(int x = i-rad; x <= i+rad; x++)
        for(int y = j-rad; y <= j+rad; y++)
            if(grid[x][y]->occupation == OCCUPIED)
                return true;
    return false;    
}

std::vector<double> PotentialGrid::normalizedGradient(int x, int y){
    std::vector<double> v(2);
    // mtx.lock();
    v[0] = (grid[x-1][y]->potential - grid[x+1][y]->potential)/2;
    v[1] = (grid[x][y-1]->potential - grid[x][y+1]->potential)/2;
    // mtx.unlock();

    if(v[0] == 0 && v[1] == 0)
        return v;

    double size_v = sqrt(pow(v[0],2) + pow(v[1],2));
    v[0] = (v[0]/size_v);
    v[1] = (v[1]/size_v);
    return v;
}

void PotentialGrid::followPotential(){
    geometry_msgs::TransformStamped pos;
    if(current_position(&pos)==-1)
        return;
   
    mtx.lock();
    int x = grid_x(pos);
    int y = grid_y(pos);
    std::vector<double> gradient = normalizedGradient(x, y);
    mtx.unlock();
    // ROS_INFO("gradient (%f, %f)", gradient[0], gradient[1]);
    if(param_pub_gradient_vec)
        publishVector(gradient, pos);
    if(param_pub_path)
        publishPath(pos);

    geometry_msgs::Twist vel;
    
    vel.linear.y = 0.00;
    vel.linear.z = 0.00;
    vel.linear.x = 0.03;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
   
    if (gradient[0] == 0 && gradient[1] == 0){
        vel.angular.z = vel.linear.x = 0.0;   
        vel.linear.x = -0.01;
    }
    else{
        float MAX_VEL = 0.7;

        tf2::Quaternion rotation;
        tf2::fromMsg(pos.transform.rotation, rotation);
        double robot_angle = tf2::getYaw(rotation); // [0, 2PI]
        // ROS_INFO("robot angle: %f", robot_angle);

        double gradient_angle = atan2(gradient[1], gradient[0]);
        if(isnan(gradient_angle)){
            // ROS_INFO("nan! gradient (%f, %f)", gradient[0], gradient[1]);
            if(gradient[1] > 0)
                gradient_angle = M_PI_2;
            else
                gradient_angle = M_PI_2;
        }        
        // ROS_INFO("gradient angle: %f", gradient_angle); 
        
        double phi = gradient_angle - robot_angle; 
        // ROS_INFO("phi: %f == %.2f", phi, normalizeAngle(phi));
        phi = normalizeAngle(phi); 

        if(phi > M_PI_2){
            vel.angular.z = 0.5;
            vel.linear.x  = 0.0;
        }
        else if(phi < -M_PI_2){
            vel.angular.z = -0.5;
            vel.linear.x  = 0.0;
        }
        else{
            vel.angular.z = (phi/M_PI_2)*0.5;
        }
    }
    vel_pub.publish(vel);
}

double PotentialGrid::normalizeAngle(double a){
    while(a > M_PI)
        a -= M_PI*2;
    while(a < -M_PI)
        a += M_PI*2;  
    return a;
}

void PotentialGrid::publishPotentialField(nav_msgs::MapMetaData info){
    nav_msgs::OccupancyGrid pf;
    pf.info = info;
    pf.header.frame_id = "map";
    pf.header.seq = 1;
    for(int i=0; i<width; i++)
        for(int j=0; j<height; j++){
            pf.data.push_back(grid[j][i]->show());
        }
    potential_pub.publish(pf);
}

void PotentialGrid::publishVector(std::vector<double> v, geometry_msgs::TransformStamped rob){
    uint32_t arrow = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "tb_grad";
    mk.id = 0;
    mk.type = arrow;
    mk.action = visualization_msgs::Marker::ADD;
   
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0; 
    mk.pose.orientation.z = 0.0; 
    mk.pose.orientation.w = 1.0;  
    mk.pose.position.x = 0.0;      
    mk.pose.position.y = 0.0; 
    mk.pose.position.z =   0.0; 
   
    mk.points.resize(2);
    mk.points[0].x = rob.transform.translation.x;
    mk.points[0].y = rob.transform.translation.y;
    mk.points[0].z = rob.transform.translation.z;
    mk.points[1].x = mk.points[0].x + v[0]*0.25;
    mk.points[1].y = mk.points[0].y + v[1]*0.25;
    mk.points[1].z = mk.points[0].z; 

    mk.color.r = 0.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.scale.x =0.02;
    mk.scale.y =0.02;
    mk.scale.z =0.10;

    mk.lifetime = ros::Duration();
    vector_pub.publish(mk);
}

void PotentialGrid::publishVectorField(){
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = ros::Time();
    m.ns = "vector_field";
    m.id = 0;
    m.type   = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = resolution*0.10;

    // m.color.r = 1.0f;
    // m.color.g = 0.0f;
    // m.color.b = 1.0f;
    // m.color.a = 1.0f;

    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.pose.position.x = -10;
    m.pose.position.y = -10;
    m.pose.position.z = 0;

    for(int i = 0; i < grid.size(); i+=2){
        for(int j = 0; j < grid[j].size(); j+=2){
            if(grid[i][j]->occupation == FREE){
                std::vector<double> g = normalizedGradient(i, j);
                geometry_msgs::Point *p0 = new geometry_msgs::Point;
                geometry_msgs::Point *pf = new geometry_msgs::Point;
                p0->x = resolution*i + resolution/2;
                p0->y = resolution*j + resolution/2;
                p0->z = 0.0;

                pf->x = p0->x + (resolution*1.0)*g[0];
                pf->y = p0->y + (resolution*1.0)*g[1];
                pf->z = 0.0;
                
                std_msgs::ColorRGBA *c0 =new std_msgs::ColorRGBA;
                std_msgs::ColorRGBA *cf =new std_msgs::ColorRGBA;
                c0->r = 1.0f;
                c0->g = 0.0f;
                c0->b = 0.0f;
                c0->a = 1.0f;
                cf->r = 1.0f;
                cf->g = 0.0f;
                cf->b = 1.0f;
                cf->a = 1.0f;                

                if(p0->x > pf->x || p0->y > pf->y){
                    m.points.push_back(*pf);
                    m.points.push_back(*p0);
                    m.colors.push_back(*cf);      
                    m.colors.push_back(*c0);  
                }
                else{
                    m.points.push_back(*p0);
                    m.points.push_back(*pf);
                    m.colors.push_back(*c0);      
                    m.colors.push_back(*cf);      
                }
            }
        }
    }
    vector_field_pub.publish(m);
}

void PotentialGrid::publishPath(geometry_msgs::TransformStamped current_pos){
    geometry_msgs::PoseStamped * p = new geometry_msgs::PoseStamped;
    p->header.frame_id = path.header.frame_id;
    p->header.stamp = current_pos.header.stamp;
    p->pose.orientation = current_pos.transform.rotation;
    p->pose.position.x = current_pos.transform.translation.x;
    p->pose.position.y = current_pos.transform.translation.y;
    p->pose.position.z = current_pos.transform.translation.z;

    path.poses.push_back(*p);
    path_pub.publish(path);
}

void PotentialGrid::publishFrontiers(std::vector<int> centers){
    nav_msgs::GridCells f;
    f.cell_height = resolution;
    f.cell_width  = resolution;
    f.header.frame_id = "map";
    f.header.stamp    = ros::Time();
    ROS_INFO("n frontiers = %d", (int)(centers.size()/2));
    for(int n=0; n<centers.size(); n+=2){
        geometry_msgs::Point p;
        p.x = centers[n];
        p.y = centers[n+1];
        p.z = 0.0;
        f.cells.push_back(p);
    }
    front_pub.publish(f);
}

Cell::Cell(int v){
    if(v >= OCC_TRESH){
        potential  = 1.0;
        occupation = OCCUPIED;
    }
    else{
        if(v >= 0 && v<= FREE_TRESH){
            occupation = FREE;
        }
        else{
            occupation = UNEXPLORED;
        }
        potential = 0.5;
    }
    frontier = NDA;
}

void Cell::update(int v){
    if(v >= OCC_TRESH){
        potential  = 1.0;
        occupation = OCCUPIED;
    }
    else{
        if(v >= 0 && v<= FREE_TRESH){
            occupation = FREE;
        }
        else{
            occupation = UNEXPLORED;
        }
    }
}
 
int Cell::show(){
    if(occupation == UNEXPLORED)
        return -1;
    return (int) (potential * 100);
}