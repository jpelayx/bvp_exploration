#include "PotentialGrid.h"

PotentialGrid::PotentialGrid(ros::NodeHandle *n)
{
    initialized = 0;
    // n->getParam("pub_potential", param_pub_pot);
    // n->getParam("pub_gradient", param_pub_gradient_vec);
    // n->getParam("pub_path", param_pub_path);
    // n->getParam("window_radius", param_window_radius);
    // n->getParam("conv_tol", param_potential_convergence_tol);
    param_potential_convergence_tol = 0.5;
    param_window_radius = 5.0;
    param_pub_pot = 1;
    param_pub_gradient_vec = 1;
    param_pub_path = 1;

    map_sub = n->subscribe("map", 1, &PotentialGrid::getMap, this);
    tf_listener = new tf2_ros::TransformListener (tf_buffer);
    
    vel_pub = n->advertise<geometry_msgs::Twist>("cmd_vel",1);

    if(param_pub_pot){
        ROS_INFO("publishing to /potential_field");
        potential_pub = n->advertise<nav_msgs::OccupancyGrid>("potential_field",1);
    }
    if(param_pub_gradient_vec){
        ROS_INFO("publishing to /gradient");
        vector_pub = n->advertise<visualization_msgs::Marker>("gradient",1);
    }
    if(param_pub_path){
        path_pub = n->advertise<nav_msgs::Path>("path",1);
        path.header.frame_id = "map";
        path.header.seq = 0;
    }
}

void PotentialGrid::getMap(const nav_msgs::OccupancyGrid::ConstPtr& map){
    height = map->info.height;
    width  = map->info.width;
    resolution = map->info.resolution;

    int map_size = height * width;
    int x0=0, xf=height, y0=0, yf=width;

    ROS_INFO("updating info");

    geometry_msgs::TransformStamped current_pos;
    if(current_position(&current_pos) == -1){
        return;
    }    
    ROS_INFO("current position = %f, %f\n height, width, resolution = %d, %d, %f", current_pos.transform.translation.x, current_pos.transform.translation.y,
                                                                                   height, width, resolution);
    current_grid_x = grid_x(current_pos);
    current_grid_y = grid_y(current_pos);
    ROS_INFO("position on grid = %d, %d", current_grid_x, current_grid_y);

    mtx.lock();
    if(!initialized){
        //new grid
        ROS_INFO("new grid");
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
        
        x0 = current_grid_x - (int)(param_window_radius/map->info.resolution);
        if(x0<0)
            x0=0;
        xf = current_grid_x + (int)(param_window_radius/map->info.resolution);
        if(xf>width)
            xf=width;
        y0 = current_grid_y - (int)(param_window_radius/map->info.resolution);
        if(y0<0)
            y0=0;
        yf = current_grid_y + (int)(param_window_radius/map->info.resolution);
        if(yf>height)
            yf=height;

        for(int i=x0; i<xf; i++){
            for(int j=y0; j<yf; j++){
                grid[i][j]->update(map->data[i+j*width]);
            }
        }
        //TODO: se nao achar fronteira update global
    }
    ROS_INFO("updating potential (%d to %d, %d to %d", x0, xf, y0, yf);
    updatePotential(x0, xf, y0, yf);
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
            current_pos = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
            *pos = current_pos;
            return 0;
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return -1;
    }
}

int PotentialGrid::grid_x(geometry_msgs::TransformStamped pos){
    return (int) (pos.transform.translation.x/resolution + width/2);
}

int PotentialGrid::grid_y(geometry_msgs::TransformStamped pos){
    return (int) (pos.transform.translation.y/resolution + height/2);
}

void PotentialGrid::updatePotential(int x, int x_max, int y, int y_max){
    setGoal(0,width,0,height);
    ROS_INFO("goals set");
    double error = param_potential_convergence_tol + 1;
    int iterations = 0;

    while(error > param_potential_convergence_tol){
        error = 0;
        for(int i=x; i<x_max; i++){
            for (int j=y; j<y_max; j++){
                if (grid.at(i).at(j)->occupation == FREE){
                    double old = grid.at(i).at(j)->potential;
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

void PotentialGrid::setGoal(int x, int x_max, int y, int y_max){
    for(int i=x; i<x_max; i++){
        for(int j=y; j<y_max; j++){
            if(isFrontier(i,j)){
                grid.at(i).at(j)->frontier  = FRONTIER;
                grid.at(i).at(j)->potential = 0.0;
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

std::vector<float> PotentialGrid::normalizedGradient(int x, int y){
    std::vector<float> v(2);
    mtx.lock();
    v[0] = (grid[x+1][y]->potential - grid[x-1][y]->potential)/2;
    v[1] = (grid[x][y+1]->potential - grid[x][y-1]->potential)/2;
    mtx.unlock();

    ROS_INFO("unormalized gradient: (%f, %f)", v[0], v[1]);
    float size_v = sqrt(pow(v[0],2) + pow(v[1],2));
    v[0] = (v[0]/size_v);
    v[1] = (v[1]/size_v);
    return v;
}

void PotentialGrid::followPotential(){
    geometry_msgs::TransformStamped current_pos;
    if(current_position(&current_pos)==-1)
        return;

    tf2::Quaternion rotation;
    tf2::fromMsg(current_pos.transform.rotation, rotation);
    double robot_angle = rotation.getAngle(); // [0, 2PI]
    robot_angle = normalizeAngle(robot_angle);
    ROS_INFO("normalized robot angle: %f", robot_angle);
    
    int x = grid_x(current_pos);
    int y = grid_y(current_pos);

    std::vector<float> gradient = normalizedGradient(x, y);
    //ROS_INFO("at (%d, %d), gradient (%f, %f)", x, y, gradient[0], gradient[1]);
    if(param_pub_gradient_vec)
        publishVector(gradient, current_pos);

    geometry_msgs::Twist vel;
    
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    
    if (gradient[0] == 0 && gradient[1] == 0)
        vel.angular.z = vel.linear.x = 0.0;
    else{
        double phi = atan2(gradient[1], gradient[2]); //atan2: [-PI, PI]
        ROS_INFO("gradient angle: %f", phi ); 
        phi = phi - robot_angle; 
        phi = normalizeAngle(phi);
        ROS_INFO("normalized phi: %f", phi)

        if(phi < -M_PI/2){
            vel.angular.z = 0.1;
            vel.linear.x  = 0.0; 
        }else if(phi > M_PI/2){
            vel.angular.z = 0.1;
            vel.linear.x  = 0.0;  
        }else {
            vel.linear.x = 0.1;
            vel.angular.z = (2*phi/M_PI)*vel.linear.x;
        }
    }
}

double PotentialGrid::normalizeAngle(double angle){
    while(angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;    
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

void PotentialGrid::publishVector(std::vector<float> v, geometry_msgs::TransformStamped robot_transform){
    uint32_t arrow = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "tb_grad";
    mk.id = 0;
    mk.type = arrow;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = robot_transform.transform.translation.x;
    mk.pose.position.y = robot_transform.transform.translation.y;
    mk.pose.position.z = robot_transform.transform.translation.z;

    tf2::Quaternion rotation;
    rotation.setEuler(0.0, asin(v[1]), 0.0);
    mk.pose.orientation = tf2::toMsg(rotation);

    mk.color.r = 0.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.scale.x =0.25;
    mk.scale.y =0.02;
    mk.scale.z =0.02;

    mk.lifetime = ros::Duration();
    vector_pub.publish(mk);
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