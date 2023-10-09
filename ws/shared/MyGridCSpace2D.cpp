#include "MyGridCSpace2D.h"
#include <Eigen/Core>
#include <math.h>
#include "LinkImplementation.h"

using namespace std;

namespace amp{

//checks for collision of the manipulator and workspace obstacles for a given manipulator configuration
bool MyGridCSpace2D::inCollision(double x0, double x1) const{

    LinkImplementation manipulator = LinkImplementation(getLinkLengths()); //generate a manipulator from the link lengths
    vector<Polygon> obstacles = getObstacles(); //get the obstacles in the workspace

    //get the positions of each of the three joints in the manipulator using forward kinematics
    vector<double> state;
    state.push_back(x0);
    state.push_back(x1);
    vector<Eigen::Vector2d> joint_pos;
    joint_pos.push_back(manipulator.getJointLocation(state,0));
    joint_pos.push_back(manipulator.getJointLocation(state,1));
    joint_pos.push_back(manipulator.getJointLocation(state,2));

    bool intersection = false; //true if the manipulator intersects an obstacle
    double tol = 0.0000001; //used for checking "equality" (i was having floating point issues)

    //loop through every obstacle
    for(int i = 0; i < obstacles.size(); i++){

        amp::Polygon obstacle = obstacles[i]; //select an obstacle
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW(); //extract the vertices in counter clockwise order
        int vert_num = vertices.size(); //store the number of vertices

        //loop through each edge of the obstacle
        for(int j = 0; j < vert_num; j++){

            //extract vertices of the edge and their corresponding x and y coordinates
            Eigen::Vector2d v1 = vertices[j]; 
            Eigen::Vector2d v2 = vertices[(j+1)%vert_num];
            double ex1 = v1[0];
            double ey1 = v1[1];
            double ex2 = v2[0];
            double ey2 = v2[1];

            for(int k = 0; k < 2; k++){ //loop through each link in the manipulator

                //extract vertices of the link and their corresponding x and y coordinates
                Eigen::Vector2d joint1 = joint_pos[k];
                Eigen::Vector2d joint2 = joint_pos[k+1];
                double lx1 = joint1[0];
                double ly1 = joint1[1];
                double lx2 = joint2[0];
                double ly2 = joint2[1];

                //define variables to store the intersection point of the current edge and the path
                double x_int;
                double y_int;

                bool parallel = false; //true if edge and link are parallel

                //First, find intersection point of arm link and edge of obstacle (will always exist if they are not parallel)

                //check if edge is vertical (slope undefined)
                if(abs(ex1 - ex2) <= tol){

                    //edge is vertical
                    //make sure link is not also vertical
                    if(abs(lx1 - lx2) > tol){
                        
                        //path is not also vertical
                        double m = (ly2-ly1)/(lx2-lx1); //find slope of the link
                        double b = ly2-m*lx2; //find y-int of the link

                        //find the intersection point of the link and the edge
                        x_int = ex1;
                        y_int = m*x_int+b;

                    }else{
                        //parallel vertical edges
                        parallel = true;
                    }


                }else if(abs(lx1 - lx2) <= tol){ //check if link is vertical

                    //path is vertical
                    //edge cannot also be vertical (already checked in previous case)

                    double m = (ey2-ey1)/(ex2-ex1); //find slope of the edge
                    double b = ey1-m*ex1; //find y-int of the edge

                    //find the intersection point of the edge and the link
                    x_int = lx2;
                    y_int = m*x_int+b;  
                    

                }else{
                    
                    //neither the edge nor the path are vertical 

                    double m1 = (ey2-ey1)/(ex2-ex1); //find slope of the edge
                    double b1 = ey1-m1*ex1; //find y-int of the edge
                    double m2 = (ly2-ly1)/(lx2-lx1); //find slope of the link
                    double b2 = ly2-m2*lx2; //find y-int of the link

                    //make sure that lines arent parallel
                    if(abs(m1 - m2) > tol){

                        //find the intersection point of the edge and the link
                        x_int = (b2-b1)/(m1-m2);
                        y_int = m1*x_int+b1;

                    }else{
                        //edges are non-vertical but still parallel
                        parallel = true;
                    }
                }

                //if edges arent parallel, intersection point exists
                if(!parallel){

                    //check location of intersection point relative to edge end points
                    bool in_edge = false;
                    double edge_x_t;
                    double edge_y_t;
                    
                    if(abs(ex1 - ex2) > tol){ //edge is not vertical, use x ratio
                        edge_x_t = (x_int - ex1)/(ex2-ex1);
                
                        //ratio must be between 0 and 1
                        if( edge_x_t >= 0 && edge_x_t <= 1){ 
                            in_edge = true; //intersection point is between edge end points
                        }
                    }else{ //edge is vertical, x ratio is not defined

                        edge_y_t = (y_int - ey1)/(ey2-ey1); //use y ratio

                        //ratio must be between 0 and 1
                        if( edge_y_t >= 0 && edge_y_t <= 1){
                            in_edge = true; //intersection point is between edge end points
                        }
                    }

                    //check location of intersection point relative to link end points
                    bool in_link = false;
                    double link_x_t = -1;
                    double link_y_t = -1;
                    
                    if( abs(lx1 - lx2) > tol){ //link is not vertical, use x ratio
                        
                        link_x_t = (x_int - lx1)/(lx2-lx1);

                        //ratio must be between 0 and 1 
                        if( link_x_t >= 0 && link_x_t <= 1){
                            in_link = true; //intersection point is between link end points
                        }

                    }else{ //link is vertical, x ratio is not defined

                        link_y_t = (y_int - ly1)/(ly2-ly1); //use y ratio

                        //ratio must be between 0 and 1
                        if( link_y_t >= 0 && link_y_t <= 1){
                            in_link = true; //intersection point is between link end points
                        }
                    }

                    //check that the intersection point is between the edge end points and between the link endpoints
                    if(in_edge && in_link){
                        
                        intersection = true; //a physical intersection has occurred

                        //exit all loops
                        k = 2;
                        j = vert_num;
                        i = obstacles.size();
                    }
                }         
            }
        }
    } 
    return intersection; //return the intersection status
}

}