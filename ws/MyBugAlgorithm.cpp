#include "MyBugAlgorithm.h"
#include <math.h>


using namespace std;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem){
    //return bug1(problem);
    return bug2(problem); //use this line to switch between bug1 and bug2 algorithms
}

amp::Path2D MyBugAlgorithm::bug1(const amp::Problem2D& problem) const{

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path; //initialize path
    path.waypoints.push_back(problem.q_init); //add start point to path

    int ob_num = (problem.obstacles).size(); //find number of obstacles
    double step_size = 0.01; //define distance of each step

    std::vector<amp::Polygon> obstacles; //array to store the obstacles in the problem
    std::vector<Eigen::Vector2d> vertices; //array to temporarily store the vertices of the obstacles
    Eigen::Vector2d vertex; //vector to temporarily store vertices
    Eigen::Vector2d center; //vector to store the center-point of each object
    Eigen::Vector2d center_offset; //vector that points from center to a vertex

    //"inflate" all object by a small amount to ensure bug does not overlap on boundary
    for(int i = 0; i < ob_num; i++){ //iterate over all object
        
        vertices = (problem.obstacles[i]).verticesCCW(); //extract the vertices of the object in CCW order

        center[0] = 0; //initialize the center coordinates
        center[1] = 0;
        
        //compute average position of all vertices - the center of the object
        for(int j = 0; j < vertices.size(); j++){ //loop through each vertex
            center = center + vertices[j]; 
        }
        center = center/(vertices.size());

        for(int j = 0; j < vertices.size(); j++){ //loop through all vertices

            center_offset = vertices[j]-center; //calculate the vector pointing from the center to the vertex
            vertices[j] = vertices[j] + 2*step_size*center_offset/center_offset.norm(); //move the vertex slightly in this direction
        }

        amp::Polygon object = amp::Polygon(vertices); //initialize a new object with these pusshed-back vertices

        obstacles.push_back(object); //add the inflated object to the array
    }

    Eigen::Vector2d curr_pos = problem.q_init; //define a vector storing the bugs current position
    Eigen::Vector2d to_goal; //define a vector that points from the current position to the goal
    bool hit = false; //stores whether an obstacle has been hit
    bool at_goal = false; //stores whether the bug is at the goal
    bool possible = true; //stores whether we believe the target is reachable
    double x; //stores current x val
    double y; //stores current y val
    double prevx; //stores previous x val
    double prevy; //stores previous y val
    amp::Polygon int_ob; //if the bug intersects an obstacle, the obstacle is stored here 
    int int_ob_num = 0; //the number of the first object intersected when a collision occurs
    int int_edge_num = 0; //the number of the first intersected edge of the first intersected obstacle
    bool loop = false; //indicates if the bug has looped around the obstacle yet
    std::vector<Eigen::Vector2d> int_vertices; //extract all vertices of the obstacle
    Eigen::Vector2d next_vertex; //store the next vertex that the bug is moving toward
    int next_vert_num; //store the number of the next vertex that the bug is moving toward
    int vert_count; //counts the number of vertices encountered since collision
    double min_dist; //stores the minimum distance to the goal while rounding an obstacle
    double tol = step_size/5; //tolerance for bug to leave obstacles
    int curr_ob_num; //stores the number of the current object being followed (if there is one)
    Eigen::Vector2d to_next_vertex; //vector pointing to the next vertex that the bug is moving toward
    int count = 0; //counts the number of loop iterations that have occurred (preven infinite loops if bug gets stuck)
    

    //main bug loop, continue until goal is reached or it is determined that it cannot be reached
    while(!at_goal && possible){

        //check if bug hit an obstacle
        if(!hit){ 
            
            //the bug did not hit an obstacle

            //bug is not at goal yet, bug ismoving toward goal
            to_goal = problem.q_goal - curr_pos; //update to goal vector
            curr_pos = curr_pos + step_size*to_goal/to_goal.norm(); //move the bug one step size in the direction of the goal
            x = curr_pos[0]; //store updated x and y values
            y = curr_pos[1];

            //check for a collision and update the position to the intersection point
            //and update int_ob_num and int_edge_num to the intersected object and edge numbers if there is one
            hit = detect_collision(obstacles, curr_pos, prevx, prevy, false, 0, int_ob_num, int_edge_num); 

            //check if a collision occured during previous step
            if(hit){
                next_vert_num = int_edge_num; //set the next vertex the bug will move towards
                curr_ob_num = int_ob_num; //set the current object that the bug is following 
                vert_count = 0; //set the vertex count to zero (no vertices encountered in this collision)
                min_dist = 1000; //set the minimum distance encountered so far to a very high value
                to_next_vertex = next_vertex-curr_pos; //direction vector pointing to next vertex
            }

        }else{
            //the bug has hit an obstacle

            //the bug must now loop around the obstacle to find the closest point to the goal
            int_ob = obstacles[curr_ob_num]; //get the current obstacle that is being followed
            int_vertices = int_ob.verticesCCW(); //extract all vertices of the obstacle
            next_vertex = int_vertices[next_vert_num]; //extract the next vertex the bug is moving toward
            to_next_vertex = next_vertex-curr_pos; //direction vector pointing to next vertex

            curr_pos = curr_pos + step_size*to_next_vertex/to_next_vertex.norm(); //move the bug one step size in the direction of the vertex
            x = curr_pos[0]; //store updated x and y values
            y = curr_pos[1];

            double curr_dist = (problem.q_goal - curr_pos).norm(); //find the current distance to the goal

            if(loop){ //check if the bug has already looped the obstacle once

                //bug has already rounded the obstacle once
                //check if bug is back at its closest point to the goal
                if( abs(curr_dist-min_dist) < tol){
                    hit = false; //if so, return to goal following mode
                    loop = false;
                }
            }else{

                //bug is still on first loop
                if(curr_dist < min_dist){ //if bug is closer to goal that minimum distance, update minimum distance
                    min_dist = curr_dist;
                }
            }

            int p_ob_num = curr_ob_num; //store the current object number in another variable for passing by reference purposes

            //check for a collision and update the position to the intersection point
            //and update int_ob_num and int_edge_num to the intersected object and edge numbers if there is one
            bool new_hit = detect_collision(obstacles, curr_pos, prevx, prevy, true, p_ob_num, curr_ob_num, next_vert_num);

            //update current obstacle, vertices, and next vertex
            int_ob = obstacles[curr_ob_num];
            int_vertices = int_ob.verticesCCW();
            next_vertex = int_vertices[next_vert_num];

            //check if the bug is within half a step size of the next vertex, if so, move to be exactly on the vertex (dont worry about this other condition here :))
            if(to_next_vertex.norm() < step_size/2 && !(((next_vertex - curr_pos).norm() > step_size) && !(curr_pos[0] == 100 && curr_pos[1] == 100))){ //check if the bug is as close as possible to the vertex
                
                //set position to the vertex position
                curr_pos = next_vertex;
                x = curr_pos[0];
                y = curr_pos[1];
    
                //update the next vertex the bug will move toward in the clockwise direction (mod wasnt working for negatives)
                next_vert_num = next_vert_num-1;
                if(next_vert_num < 0){ 
                    next_vert_num = int_vertices.size()-1;
                }
                
                vert_count++; //increment the vertex count   
            }

            //check if a loop has been completed yet (are we back on the same object and edge we started on?)
            if(vert_count > 0 && next_vert_num == int_edge_num && curr_ob_num == int_ob_num){ //check if the bug has completed a full loop
                loop = true;
            }

            //fix the position if something really weird occurs 
            Eigen::Vector2d prev_pos;
            prev_pos[0] = prevx;
            prev_pos[1] = prevy;
            if(curr_pos[0] == 100 || curr_pos[1] == 100){
                curr_pos = prev_pos + step_size*to_next_vertex/to_next_vertex.norm();
            }  
        }

        path.waypoints.push_back(curr_pos);//add the bugs new position to the path
        prevx = curr_pos[0]; //update previous position variables
        prevy = curr_pos[1]; 

        //escape loop if infinite loop
        if(count >100000){
            possible = false;
        }
        count++;
        
        if(to_goal.norm() < step_size/2){ //check if the bug is as close as possible to the goal given the step size
            at_goal = true; //bug is at goal, exit loop, success
        }
    }

    path.waypoints.push_back(problem.q_goal); //add the goal to the path
    double length = path_length(path); //compute the length of the path

    return path; //return the path
}

amp::Path2D MyBugAlgorithm::bug2(const amp::Problem2D& problem) const{
    
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path; //initialize path
    path.waypoints.push_back(problem.q_init); //add start point to path

    int ob_num = (problem.obstacles).size(); //find number of obstacles
    double step_size = 0.01; //define distance of each step

    std::vector<amp::Polygon> obstacles; //array to store the obstacles in the problem
    std::vector<Eigen::Vector2d> vertices; //array to temporarily store the vertices of the obstacles
    Eigen::Vector2d vertex; //vector to temporarily store vertices
    Eigen::Vector2d center; //vector to store the center-point of each object
    Eigen::Vector2d center_offset; //vector that points from center to a vertex

    //"inflate" all object by a small amount to ensure bug does not overlap on boundary
    for(int i = 0; i < ob_num; i++){ //iterate over all object
        
        vertices = (problem.obstacles[i]).verticesCCW(); //extract the vertices of the object in CCW order

        center[0] = 0; //initialize the center coordinates
        center[1] = 0;
        
        //compute average position of all vertices - the center of the object
        for(int j = 0; j < vertices.size(); j++){ //loop through each vertex
            center = center + vertices[j]; 
        }
        center = center/(vertices.size());

        for(int j = 0; j < vertices.size(); j++){ //loop through all vertices

            center_offset = vertices[j]-center; //calculate the vector pointing from the center to the vertex
            vertices[j] = vertices[j] + 2*step_size*center_offset/center_offset.norm(); //move the vertex slightly in this direction
        }

        amp::Polygon object = amp::Polygon(vertices); //initialize a new object with these pusshed-back vertices

        obstacles.push_back(object); //add the inflated object to the array
    }

    Eigen::Vector2d curr_pos = problem.q_init; //define a vector storing the bugs current position
    Eigen::Vector2d to_goal; //define a vector that points from the current position to the goal
    bool hit = false; //stores whether an obstacle has been hit
    bool at_goal = false; //stores whether the bug is at the goal
    bool possible = true; //stores whether we believe the target is reachable
    double x; //stores current x val
    double y; //stores current y val
    double prevx; //stores previous x val
    double prevy; //stores previous y val
    amp::Polygon int_ob; //if the bug intersects an obstacle, the obstacle is stored here 
    int int_ob_num = 0; //the number of the first object intersected when a collision occurs
    int int_edge_num = 0; //the number of the first intersected edge of the first intersected obstacle
    std::vector<Eigen::Vector2d> int_vertices; //extract all vertices of the obstacle
    Eigen::Vector2d next_vertex; //store the next vertex that the bug is moving toward
    int next_vert_num; //store the number of the next vertex that the bug is moving toward
    int vert_count; //counts the number of vertices encountered since collision
    double min_dist; //stores the minimum distance to the goal while rounding an obstacle
    double tol = step_size/5; //tolerance for bug to leave obstacles
    int curr_ob_num; //stores the number of the current object being followed (if there is one)
    Eigen::Vector2d to_next_vertex; //vector pointing to the next vertex that the bug is moving toward
    int count = 0; //counts the number of loop iterations that have occurred (preven infinite loops if bug gets stuck)
    double m_slope = (problem.q_goal[1]-problem.q_init[1])/(problem.q_goal[0]-problem.q_init[0]); //slope of the m-line (line to goal)
    double m_int = problem.q_goal[1]-m_slope*problem.q_goal[0]; //y-intercept of the m-line
    double hit_dist; //distance that an obstacle is first encountered at

    //main bug loop, continue until goal is reached or it is determined that it cannot be reached
    while(!at_goal && possible){

        //check if bug hit an obstacle
        if(!hit){ 
            
            //the bug did not hit an obstacle

            //bug is not at goal yet, bug ismoving toward goal
            to_goal = problem.q_goal - curr_pos; //update to goal vector
            curr_pos = curr_pos + step_size*to_goal/to_goal.norm(); //move the bug one step size in the direction of the goal
            x = curr_pos[0]; //store updated x and y values
            y = curr_pos[1];

            //check for a collision and update the position to the intersection point
            //and update int_ob_num and int_edge_num to the intersected object and edge numbers if there is one
            hit = detect_collision(obstacles, curr_pos, prevx, prevy, false, 0, int_ob_num, int_edge_num); 

            //check if a collision occured during previous step
            if(hit){
                next_vert_num = int_edge_num; //set the next vertex the bug will move towards
                curr_ob_num = int_ob_num; //set the current object that the bug is following 
                hit_dist = (curr_pos-problem.q_goal).norm(); //update the hit_dist to the distance that the collision occurred at
                to_next_vertex = next_vertex-curr_pos; //direction vector pointing to next vertex
            }

        }else{
            //the bug has hit an obstacle

            //the bug must now loop around the obstacle to find the closest point to the goal
            int_ob = obstacles[curr_ob_num]; //get the current obstacle that is being followed
            int_vertices = int_ob.verticesCCW(); //extract all vertices of the obstacle
            next_vertex = int_vertices[next_vert_num]; //extract the next vertex the bug is moving toward
            to_next_vertex = next_vertex-curr_pos; //direction vector pointing to next vertex

            curr_pos = curr_pos + step_size*to_next_vertex/to_next_vertex.norm(); //move the bug one step size in the direction of the vertex
            x = curr_pos[0]; //store updated x and y values
            y = curr_pos[1];

            double curr_dist = (problem.q_goal - curr_pos).norm(); //find the current distance to the goal

            //check if the bug just crossed the m-line and that it is closer to the goal than when it first hit the obstacle
            if(curr_dist < hit_dist &&  ( ( (y-m_slope*x-m_int) < 0 && (prevy-m_slope*prevx-m_int) > 0) || ( (y-m_slope*x-m_int) > 0 && (prevy-m_slope*prevx-m_int) < 0) || y-m_slope*x-m_int == 0)){
                hit = false; //return to goal following mode
            }

            int p_ob_num = curr_ob_num; //store the current object number in another variable for passing by reference purposes

            //check for a collision and update the position to the intersection point
            //and update int_ob_num and int_edge_num to the intersected object and edge numbers if there is one
            bool new_hit = detect_collision(obstacles, curr_pos, prevx, prevy, true, p_ob_num, curr_ob_num, next_vert_num);

            //update current obstacle, vertices, and next vertex
            int_ob = obstacles[curr_ob_num];
            int_vertices = int_ob.verticesCCW();
            next_vertex = int_vertices[next_vert_num];

            //check if the bug is within half a step size of the next vertex, if so, move to be exactly on the vertex (dont worry about this other condition here :))
            if(to_next_vertex.norm() < step_size/2 && !(((next_vertex - curr_pos).norm() > step_size) && !(curr_pos[0] == 100 && curr_pos[1] == 100))){ //check if the bug is as close as possible to the vertex
                
                //set position to the vertex position
                curr_pos = next_vertex;
                x = curr_pos[0];
                y = curr_pos[1];
    
                //update the next vertex the bug will move toward in the clockwise direction (mod wasnt working for negatives)
                next_vert_num = next_vert_num-1;
                if(next_vert_num < 0){ 
                    next_vert_num = int_vertices.size()-1;
                }
                
                vert_count++; //increment the vertex count   
            }

        }

        //fix the position if something really weird occurs 
        Eigen::Vector2d prev_pos;
        prev_pos[0] = prevx;
        prev_pos[1] = prevy;
        if(curr_pos[0] == 100 || curr_pos[1] == 100){
            curr_pos = prev_pos + step_size*to_next_vertex/to_next_vertex.norm();
        }  

        path.waypoints.push_back(curr_pos);//add the bugs new position to the path
        prevx = curr_pos[0]; //update previous position variables
        prevy = curr_pos[1]; 

        //escape loop if infinite loop
        if(count >100000){
            possible = false;
        }
        count++;
        
        if(to_goal.norm() < step_size/2){ //check if the bug is as close as possible to the goal given the step size
            at_goal = true; //bug is at goal, exit loop, success
        }
    }

    path.waypoints.push_back(problem.q_goal); //add the goal to the path
    double length = path_length(path); //compute the length of the path

    return path; //return the path
}

//detects if the bug has collided with an obstacle, updates the position to the intersection point, updates the numbers of the current obstacle and edge being followed
bool MyBugAlgorithm::detect_collision(std::vector<amp::Polygon> obstacles, Eigen::Vector2d& curr_pos, double prevx, double prevy, bool on_object, int curr_int_ob_num, int& int_ob_num, int& int_edge_num) const{

    bool outside_all_objects = true; //if true, the bug is not inside any obstacle
    double x = curr_pos[0]; //store the current x and y position values
    double y = curr_pos[1];

    //loop through every obstacle
    for(int i = 0; i < obstacles.size(); i++){
        
        //dont check for collisions with an obstacle that the bug is already following, if the bug is following and obstacle
        if(!on_object || (on_object && i != curr_int_ob_num)){

            amp::Polygon obstacle = obstacles[i]; //select an obstacle
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW(); //extract the vertices in counter clockwise order
            int vert_num = vertices.size(); //store the number of vertices
            bool inside = true; //if true, the bug is inside the current obstacle

            //loop through each pair of vertices (each edge)
            for(int j = 0; j < vert_num; j++){

                //extract vertices and their corresponding x and y coordinates
                Eigen::Vector2d v1 = vertices[j]; 
                Eigen::Vector2d v2 = vertices[(j+1)%vert_num];
                double x1 = v1[0];
                double y1 = v1[1];
                double x2 = v2[0];
                double y2 = v2[1];

                //check to see if the line is vertical (equal x-values)
                if(x1 == x2){
                    
                    //check if edge is moving up or down (compare y values)
                    if(y1 > y2){
                        
                        //because vertices listed CCW, this must be a left edge
                        //check if bug is to the left of the left edge
                        if(x < x1){
                            inside = false; //must be outside the object
                        }
                    }else{

                        //because vertices listed CCW, this must be a right edge
                        //check if bug is to the right of the right edge
                        if(x > x1){
                            inside = false; //bug must be outside the object
                        }
                    }

                }else{

                    //line is not vertical
                    double m = (y2-y1)/(x2-x1); //find slope of the line
                    double b = y1-m*x1; //find y-int of the line

                    //case one: line pointing from right to left
                    if(x1 > x2){
                        //check appropriate inequality
                        if(y-m*x > b){
                            inside = false; //bug must be outside object
                        }
                    }

                    //case two: line pointing from left to right
                    if(x1 < x2){
                        //check appropriate inequality
                        if(y-m*x < b){
                            inside = false; //bug must be outside object

                        }
                    }
                }  
            }

            //check if bug is now inside the current object 
            if(inside){

                outside_all_objects = false; //bug is inside the current object being considered

                //find the intersection point of the bugs path with the object

                //variables with x and y coordinates of true intersection point of the object with path
                double x_int_global = 100;
                double y_int_global = 100;

                //loop through each pair of vertices (each edge)
                for(int j = 0; j < vert_num; j++){

                    //extract vertices and their corresponding x and y coordinates
                    Eigen::Vector2d v1 = vertices[j]; 
                    Eigen::Vector2d v2 = vertices[(j+1)%vert_num];
                    double x1 = v1[0];
                    double y1 = v1[1];
                    double x2 = v2[0];
                    double y2 = v2[1];

                    //define variables to store the intersection point of the current edge and the path
                    double x_int_local;
                    double y_int_local;

                    //check if edge is vertical (slope undefined)
                    if(x1 == x2){

                        //edge is vertical
                        //make sure path is not also vertical
                        if(x != prevx){
                            
                            //path is not also vertical
                            double m = (y-prevy)/(x-prevx); //find slope of the bugs path
                            double b = y-m*x; //find y-int of the bugs path

                            //find the intersection point of the edge and the path
                            x_int_local = x1;
                            y_int_local = m*x_int_local+b;

                        }


                    }else if(x == prevx){ //check if path is vertical

                        //path is vertical
                        //edge cannot also be vertical (already checked in previous case)

                        double m = (y2-y1)/(x2-x1); //find slope of the edge
                        double b = y1-m*x1; //find y-int of the bugs path

                        //find the intersection point of the edge and the path
                        x_int_local = x;
                        y_int_local = m*x_int_local+b;  
                        

                    }else{
                        
                        //neither the edge nor the path are vertical 

                        double m1 = (y2-y1)/(x2-x1); //find slope of the edge
                        double b1 = y1-m1*x1; //find y-int of the edge
                        double m2 = (y-prevy)/(x-prevx); //find slope of the bugs path
                        double b2 = y-m2*x; //find y-int of the bugs path

                        //make sure that lines arent parallel
                        if(m1 != m2){

                            //find the intersection point of the edge and the path
                            x_int_local = (b2-b1)/(m1-m2);
                            y_int_local = m1*x_int_local+b1;

                        }
                    }

                    //check if distance between the intersection point and the current position is less than the current minimum
                    if(pow(x_int_local-x,2)+pow(y_int_local-y,2) < pow(x_int_global-x,2)+pow(y_int_global-y,2)){

                        if(x == prevx){ //check if path is vertical

                            //make sure that the intersection point is between the previous and current position
                            if( (y_int_local-prevy)/(y-prevy) >= 0 && (y_int_local-prevy)/(y-prevy) <= 1){
                                
                                //update intersection point values
                                x_int_global = x_int_local;
                                y_int_global = y_int_local;
                                int_ob_num = i; //update the intersected object and edge values for identification later
                                int_edge_num = j;
                            }
                            
                                
                        }else{

                            //path is not vertical

                            //make sure that the intersection point is between the previous and current position
                            if( (x_int_local-prevx)/(x-prevx) >= 0 && (x_int_local-prevx)/(x-prevx) <= 1){

                                //update intersection point values
                                x_int_global = x_int_local;
                                y_int_global = y_int_local;
                                int_ob_num = i; //update the intersected object and edge values for identification later
                                int_edge_num = j;

                            }
                        }
                    }
                }

                //update current position to closest intersection point
                curr_pos[0] = x_int_global; 
                curr_pos[1] = y_int_global;

            } 
        }
    } 

    return !outside_all_objects;
}

//determines the length of a path by adding up the norm off the difference of each consecutive waypoint vector
double MyBugAlgorithm::path_length(amp::Path2D path) const{
    std::vector<Eigen::Vector2d> waypoints = path.waypoints;
    Eigen::Vector2d v1;
    Eigen::Vector2d v2;
    double length = 0;

    for(int i = 0; i < waypoints.size()-2; i++){
        v1 = waypoints[i];
        v2 = waypoints[i+1];
        length += (v2-v1).norm();
    }

    return length;
}