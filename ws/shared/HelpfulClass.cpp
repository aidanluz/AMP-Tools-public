#include "HelpfulClass.h"
#include <math.h>

using namespace std;

amp::Polygon MyClass::C_space_obst(amp::Polygon ob, amp::Polygon rob) const{
    // Implementation

    //extract the vertices of the obstacle and robot
    std::vector<Eigen::Vector2d> ob_vert = ob.verticesCCW();
    std::vector<Eigen::Vector2d> rob_vert = rob.verticesCCW();
    std::vector<Eigen::Vector2d> cob_vert; //vertices of c-space obstacle
    std::vector<Eigen::Vector2d> cob_edges; //edges of c-space obstacle
    Eigen::Vector2d edge;
    Eigen::Vector2d norm;
    Eigen::Vector2d offset_vect1; //offset vectors to move the obstacle to the location corresponding to the tracing vertex being the (0,0) vertex of the robot
    Eigen::Vector2d offset_vect2;
    double cob_ang[ob_vert.size()+rob_vert.size()];

    //store all CCW edges of the obstacle in the c-space objects edges
    for(int i = 0; i < ob_vert.size(); i++){
        edge = ob_vert[(i+1)%ob_vert.size()]-ob_vert[i];
        cob_edges.push_back(edge);
        norm[0] = edge[1]; 
        norm[1] = -edge[0];
        cob_ang[i] = fmod(atan2(norm[1],norm[0])+ 2*M_PI,2*M_PI); //store the angle of the normal vector to the edge
        if(i == 0){
            offset_vect1 = edge; //offset vector one is the first CCW edge of the obstacle
        }
    }

    //store all CW edges of the robot in the c-space objects edges
    for(int i = 0; i < rob_vert.size(); i++){
        edge = rob_vert[i]-rob_vert[(i+1)%rob_vert.size()];
        cob_edges.push_back(edge);
        norm[0] = edge[1];
        norm[1] = -edge[0];
        cob_ang[i+ob_vert.size()]  = fmod(atan2(norm[1],norm[0])+ 2*M_PI,2*M_PI); //store the angle of the normal vector to the edge
    }
    
    //sort the edges of the c-space object in order of increasing normal vector angle
    //Then generate vertices of the c-space object, starting point will be at (0,0)
    for(int i = 0; i < cob_edges.size(); i++){

        //sorting
        for(int j = cob_edges.size()-1; j > i; j--){

            if(cob_ang[j] < cob_ang[j-1]){
                double temp = cob_ang[j-1];
                cob_ang[j-1] = cob_ang[j];
                cob_ang[j] = temp;

                Eigen::Vector2d temp_edge = cob_edges[j-1];
                cob_edges[j-1] = cob_edges[j];
                cob_edges[j] = temp_edge;
            }
        }

        //generating vertices   
        if(i == 0){
            cob_vert.push_back(cob_edges[0]);
        }else{
            cob_vert.push_back(cob_vert[i-1]+cob_edges[i]);
        }
    }

    Eigen::Vector2d first_edge = cob_edges[0]; //store the c-space obstacle edge with the smallest normal vector angle

    //generate the second offset vector (dont ask where this came from)
    for(int i = 0; i < rob_vert.size(); i++){

        Eigen::Vector2d v1 = rob_vert[i];
        Eigen::Vector2d v2 = rob_vert[(i+1)%rob_vert.size()];
        edge = v1-v2;
        
        //find the vertices of the edge with the smallest normal vector angle
        if( abs(edge[0]-first_edge[0]) < 0.0001  && abs(edge[1]-first_edge[1]) < 0.0001){
            offset_vect2 = -v1; //the second offset vector is the negative of the first vertex of the edge with the smallest normal vector (DONT ASK)
        }
    }

    //translate all vertices by the two offset vectors so that starting point of the object corresponds to tracing with the lower left vertex
    for(int i = 0; i < cob_vert.size(); i++){
        cob_vert[i] = cob_vert[i] + offset_vect1 + offset_vect2;
    }

    //construct and return the c-space obstacle
    amp::Polygon cob = amp::Polygon(cob_vert);
    return cob;
}
