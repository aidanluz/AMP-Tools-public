#include "MyBugAlgorithm.h"
#include <math.h>


using namespace std;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path; //initialize path
    path.waypoints.push_back(problem.q_init); //add start point to path

    //std::vector<amp::Polygon> obstacles = problem.obstacles; //extract list of obstacles

    int ob_num = (problem.obstacles).size(); //find number of obstacles
    double step_size = 0.01; //define distance of each step

    std::vector<amp::Polygon> obstacles;
    std::vector<Eigen::Vector2d> vertices;
    Eigen::Vector2d vertex;
    Eigen::Vector2d center;
    Eigen::Vector2d center_offset;

    for(int i = 0; i < ob_num; i++){
        
        vertices = (problem.obstacles[i]).verticesCCW();
        //vertices = (problem.obstacles[i]).verticesCW();
        //vertices.pop_back();
        center[0] = 0;
        center[1] = 0;

        for(int j = 0; j < vertices.size(); j++){
            center = center + vertices[j];
            if(i == 0){
                cout << "Object " << i << endl;
                cout << "Vertex " << j << endl;
                cout << vertices[j] << endl << endl;
            }
        }

        center = center/(vertices.size());

        for(int j = 0; j < vertices.size(); j++){

            center_offset = center-vertices[j];
            double offx = center_offset[0];
            double offy = center_offset[1];
            if(offx < 0){
                vertices[j][0] = vertices[j][0] + step_size;
            }else if(offx > 0){
                vertices[j][0] = vertices[j][0] - step_size;
            }
            
            if(offy < 0){
                vertices[j][1] = vertices[j][1] + step_size;
            }else if(offy > 0){
                vertices[j][1] = vertices[j][1] - step_size;
            }
        }

        amp::Polygon object = amp::Polygon(vertices);

        obstacles.push_back(object);

    }


    /*for(int i = 0; i < ob_num; i++){
        amp::Polygon ob = obstacles[i];
        vertices = ob.verticesCCW();
        cout << i << endl;
        for(int j = 0; j < vertices.size(); j++){
            vertex = vertices[j];
            cout << vertex << endl << endl;
        }
    }*/

    //obstacles = problem.obstacles;
    //((obstacles[0]).verticesCCW()).pop_back();


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
    int int_ob_num = 0; //the number of the intersected obstacle in the obstacles array
    int int_edge_num = 0; //the number of the intersected edge of that obstacle
    bool loop = false; //indicates if the bug has looped around the obstacle yet
    std::vector<Eigen::Vector2d> int_vertices; //extract all vertices of the obstacle
    Eigen::Vector2d next_vertex;
    int next_vert_num;
    int vert_count;
    double min_dist; //stores the minimum distance to the goal while rounding an obstacle
    double tol = step_size/5;
    int curr_ob_num;
    Eigen::Vector2d to_next_vertex;
    int count = 0;
    

    //main bug loop, continue until goal is reached or it is determined that it cannot be reached
    while(!at_goal && possible){

        to_goal = problem.q_goal - curr_pos; //update to goal vector

        if(!hit){ 
            
            //the bug did not hit an obstacle

            //bug is not at goal yet
            curr_pos = curr_pos + step_size*to_goal/to_goal.norm(); //move the bug one step size in the direction of the goal
            x = curr_pos[0]; //store updated x and y values
            y = curr_pos[1];
            hit = detect_collision(obstacles, curr_pos, prevx, prevy, false, 0, int_ob_num, int_edge_num);

            if(hit){
                int_ob = obstacles[int_ob_num];
                int_vertices = int_ob.verticesCCW(); //extract all vertices of the obstacle
                //int_vertices = int_ob.verticesCW();
                //int_vertices = ob_array[int_ob_num];
                next_vertex = int_vertices[int_edge_num];
                to_next_vertex = next_vertex - curr_pos;
                next_vert_num = int_edge_num;
                curr_ob_num = int_ob_num;
                vert_count = 0;
                min_dist = 1000;
            }

        }else{
            //the bug has hit an obstacle
            //the bug must now loop around the obstacle to find the closest point to the goal
            int_ob = obstacles[curr_ob_num];
            int_vertices = int_ob.verticesCCW(); //extract all vertices of the obstacle
            //int_vertices = int_ob.verticesCW(); 
            //int_vertices = ob_array[curr_ob_num];
            next_vertex = int_vertices[next_vert_num];
            to_next_vertex = next_vertex-curr_pos; //direction vector pointing to next vertex

            //otherwise, continue toward next vertex
            curr_pos = curr_pos + step_size*to_next_vertex/to_next_vertex.norm(); //move the bug one step size in the direction of the vertex
            x = curr_pos[0]; //store updated x and y values
            y = curr_pos[1];

            double curr_dist = (problem.q_goal - curr_pos).norm();

            if(loop){ //check if the bug has already looped the obstacle once

                //bug has already rounded the obstacle once
                //check if bug is back at its closest point to the goal
                if( abs(curr_dist-min_dist) < tol){
                    hit = false;
                    loop = false;
                }

            }else{
                //bug is still on first loop
                if(curr_dist < min_dist){ //if bug is closest to goal yet, update minimum distance
                    min_dist = curr_dist;
                }

            }

            int p_ob_num = curr_ob_num;

            bool new_hit = detect_collision(obstacles, curr_pos, prevx, prevy, true, p_ob_num, curr_ob_num, next_vert_num);

            int_ob = obstacles[curr_ob_num];
            int_vertices = int_ob.verticesCCW();
            //int_vertices = int_ob.verticesCW();  //extract all vertices of the obstacle
            //int_vertices = ob_array[curr_ob_num];
            next_vertex = int_vertices[next_vert_num];
    
            if(to_next_vertex.norm() < step_size/2 && !(((next_vertex - curr_pos).norm() > step_size) && !(curr_pos[0] == 100 && curr_pos[1] == 100))){ //check if the bug is as close as possible to the vertex
                //set position to the vertex position
                
                curr_pos = next_vertex;
                x = curr_pos[0];
                y = curr_pos[1];
                
                next_vert_num = (next_vert_num-1)%int_vertices.size(); //update vertex that the bug is moving towards
                vert_count++;
            }

            if(vert_count > 0 && next_vert_num == int_edge_num && curr_ob_num == int_ob_num){ //check if the bug has completed a full loop
                loop = true;
            }

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

        if(count > 100000){
            at_goal = true;
        }
        count++;
        
        if(to_goal.norm() < step_size/2){ //check if the bug is as close as possible to the goal given the step size
            at_goal = true; //bug is at goal, exit loop, success
        }
    }

    path.waypoints.push_back(problem.q_goal);

    return path;
}

bool MyBugAlgorithm::detect_collision(std::vector<amp::Polygon> obstacles, Eigen::Vector2d& curr_pos, double prevx, double prevy, bool on_object, int curr_int_ob_num, int& int_ob_num, int& int_edge_num) const{
//bool MyBugAlgorithm::detect_collision(std::vector<Eigen::Vector2d> ob_array, int ob_num, Eigen::Vector2d& curr_pos, double prevx, double prevy, bool on_object, int curr_int_ob_num, int& int_ob_num, int& int_edge_num) const{
    //check if the bug is in an obstacle

    bool outside_all_objects = true; //if true, the bug is not inside any obstacle
    double x = curr_pos[0];
    double y = curr_pos[1];

    //loop through every obstacle
    //for(int i = 0; i < obstacles.size(); i++){
    for(int i = 0; i < obstacles.size(); i++){
        
        //dont check for collisions with an obstacle that the bug is already following, if the bug is following and obstacle
        if(!on_object || (on_object && i != curr_int_ob_num)){

            amp::Polygon obstacle = obstacles[i]; //select an obstacle
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
            //std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW(); //extract all vertices of the obstacle
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

                        }else{

                        }
                    }

                    /*if(x > 2.6 && x < 3 && y > 2.6 && y < 3){
                        //cout << m << endl;
                        cout << b << endl;
                        //cout << x << " " << y << endl;
                        cout << "outside" << endl;
                        cout << x1 << " " << x2 << " " << j << endl;
                        cout << y-m*x << endl << endl;
                    }*/

                }
                
            }

            //check if bug is now inside the current object 
            if(inside){

                //cout << "hi" << endl;

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

                            /*if(x_int_local == 12 && y_int_local == 6){
                                cout << x1 << " " << y1 << endl;
                                cout << x2 << " " << y2 << endl;
                                cout << j << endl;
                            }  */
                        }/*else{
                            //find the intersection point of the edge and the path
                            x_int_local = x1;
                            y_int_local = y1;
                            
                        }*/


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
                                int_ob_num = i;
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

                curr_pos[0] = x_int_global;
                curr_pos[1] = y_int_global;

            } 
        }
    } 

    return !outside_all_objects;
}
