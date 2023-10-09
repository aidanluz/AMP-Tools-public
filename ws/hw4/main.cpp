// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

#include <math.h>

#include "MyGridCSpace2DConstructor.h"

#include "LinkImplementation.h"

using namespace std;

using namespace amp;

int main(int argc, char** argv) {
    /*Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    
    double pi = M_PI;
    
    //problem 1a and 1b
    
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //create 2 new environments
    amp::Environment2D env;
    amp::Environment2D env2;
    env.x_min = -5;
    env.x_max = 5;
    env.y_min = -5;
    env.y_max = 5;
    env2.x_min = -5;
    env2.x_max = 5;
    env2.y_min = -5;
    env2.y_max = 5;

    //create the triangle obsstacle from Problem 1
    std::vector<Eigen::Vector2d> vertices;
    Eigen::Vector2d v1 = Eigen::Vector2d(0,0);
    Eigen::Vector2d v2 = Eigen::Vector2d(1,2);
    Eigen::Vector2d v3 = Eigen::Vector2d(0,2);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    amp::Polygon object1 = amp::Polygon(vertices); //obstacle
    amp::Polygon object2; //robot

    //increment the angle of rotation from 0-2*pi in steps of 12
    for(double dtheta = 0; dtheta < 2*pi; dtheta += pi/6){

        //rotate the triangle shape by the current angle
        for(int i = 0; i < vertices.size(); i++){
            v1 = vertices[i];
            double theta = atan2(v1[1],v1[0]);
            double r = sqrt(v1[0]*v1[0]+v1[1]*v1[1]);
            double theta_p = theta+dtheta;
            (vertices[i])[0] = r*cos(theta_p);
            (vertices[i])[1] = r*sin(theta_p);
        }
        
        object2 = amp::Polygon(vertices); //store the rotated polygon to act as the robot
        
        //generate the c-space obstacle from the robot and the obstacle and add it to the environment
        MyClass helper;
        amp::Polygon cob = helper.C_space_obst(object1,object2);
        (env.obstacles).push_back(cob);

        //special case: when rotation = 0 rad, create a seperate environment/figure
        if(dtheta == 0){
            (env2.obstacles).push_back(cob);
        }
    }

    //create figures for the 1a and 1b problems 
    Visualizer::makeFigure(env2);
    Visualizer::makeFigure(env);


    //Problem 2a

    //initiate a manipulator and its state
    vector<double> link_lengths1;
    link_lengths1.push_back(0.5);
    link_lengths1.push_back(1);
    link_lengths1.push_back(0.5);
    ManipulatorState angles1;
    angles1.push_back(pi/6);
    angles1.push_back(pi/3);
    angles1.push_back(7*pi/4);
    LinkImplementation imp1 = LinkImplementation(link_lengths1);

    //Test forward kinematics: store the joint location for the given state
    Eigen::Vector2d jointLocation = imp1.getJointLocation(angles1,3);

    Visualizer::makeFigure(imp1,angles1); //create a figure showing the manipulator state

    //problem 2B

    //create new manipulator
    vector<double> link_lengths2;
    link_lengths2.push_back(1);
    link_lengths2.push_back(0.5);
    link_lengths2.push_back(1);
    LinkImplementation imp2 = LinkImplementation(link_lengths2);

    Eigen::Vector2d end_effector_location = Eigen::Vector2d(2,0); //store the desired location of the end effector

    //Test inverse kinematics, find the state required to reach the given end effector location
    ManipulatorState angles2 = imp2.getConfigurationFromIK(end_effector_location);

    Visualizer::makeFigure(imp2,angles2); //create a figure showing the manipulatore in its current state

    //Problem 3a

    //generate the given manipulator and workspace 
    vector<double> link_lengths3;
    link_lengths3.push_back(1);
    link_lengths3.push_back(1);
    LinkImplementation imp3 = LinkImplementation(link_lengths3);
    Environment2D env3 = HW4::getEx3Workspace1();
    env3.x_min = -2;
    env3.x_max = 2;
    env3.y_min = 0;
    env3.y_max = 4;

    //create a c-space from the given manipulator and workspace
    MyGridCSpace2DConstructor Constructor1;
    std::unique_ptr<amp::GridCSpace2D> c_space1 = Constructor1.construct(imp3,env3);

    //create figures for the workspace and c-space
    Visualizer::makeFigure(env3);
    Visualizer::makeFigure(*c_space1);

    //Problem 3b

    //generate the given workspace
    Environment2D env4; // = HW4::getEx3Workspace2();
    vector<Eigen::Vector2d> vertices1;
    vertices1.push_back(Eigen::Vector2d(0.25,1.1));
    vertices1.push_back(Eigen::Vector2d(0.25,2));
    vertices1.push_back(Eigen::Vector2d(-0.25,2));
    vertices1.push_back(Eigen::Vector2d(-0.25,1.1));
    Polygon obstacle1 = Polygon(vertices1);
    vector<Eigen::Vector2d> vertices2;
    vertices2.push_back(Eigen::Vector2d(2,-2));
    vertices2.push_back(Eigen::Vector2d(2,-1.8));
    vertices2.push_back(Eigen::Vector2d(-2,-1.8));
    vertices2.push_back(Eigen::Vector2d(-2,-2));
    Polygon obstacle2 = Polygon(vertices2);
    (env4.obstacles).push_back(obstacle1);
    (env4.obstacles).push_back(obstacle2);
    env4.x_min = -3;
    env4.x_max = 3;
    env4.y_min = -3;
    env4.y_max = 3;

    //create a c-space from the given workspace and manipulator
    MyGridCSpace2DConstructor Constructor2;
    std::unique_ptr<amp::GridCSpace2D> c_space2 = Constructor2.construct(imp3,env4);

    //create figures for the workspace and the c-space
    Visualizer::makeFigure(env4);
    Visualizer::makeFigure(*c_space2);

    //Problem 3c

    //generate the given workspace
    Environment2D env5; // = HW4::getEx3Workspace3();
    vector<Eigen::Vector2d> vertices3;
    vertices3.push_back(Eigen::Vector2d(2,-0.5));
    vertices3.push_back(Eigen::Vector2d(2,-0.3));
    vertices3.push_back(Eigen::Vector2d(-2,-0.3));
    vertices3.push_back(Eigen::Vector2d(-2,-0.5));
    Polygon obstacle3 = Polygon(vertices3);
    (env5.obstacles).push_back(obstacle1);
    (env5.obstacles).push_back(obstacle3);
    env5.x_min = -3;
    env5.x_max = 3;
    env5.y_min = -3;
    env5.y_max = 3;

    //construct a c-space from the given manipulator and environment
    MyGridCSpace2DConstructor Constructor3;
    std::unique_ptr<amp::GridCSpace2D> c_space3 = Constructor3.construct(imp3,env5);

    //generate figures for the workspace and c-space
    Visualizer::makeFigure(env5);
    Visualizer::makeFigure(*c_space3);

    Visualizer::showFigures(); //show all figures
    
    //grade the c-space constructor function
    MyGridCSpace2DConstructor constructor;

    // Grade method
    //amp::HW4::grade<LinkImplementation>(constructor, "ailu9881@colorado.edu", argc, argv);
    return 0;
}