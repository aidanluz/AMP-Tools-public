#include "LinkImplementation.h"
#include <math.h>

using namespace std;

namespace amp {
/******* User Implemented Methods ********/

/// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
/// @param state Joint angle state (radians). Must have size() == nLinks()
/// @param joint_index Joint index in order of base to end effector 
/// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
/// @return Joint coordinate
Eigen::Vector2d LinkImplementation::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{

    Eigen::Vector2d jointLocation = getBaseLocation(); //start location at (0,0)
    vector<double> link_lengths = getLinkLengths(); //store link lengths
    double angle = 0; //initialize angle at zero

    //iterate up to joint index
    for(int i = 0; i < joint_index; i++){
        angle += state[i]; //increment the nagle 
        jointLocation[0] = jointLocation[0] + link_lengths[i]*cos(angle); //add the vector of the current link the the joint position
        jointLocation[1] = jointLocation[1] + link_lengths[i]*sin(angle);
    }

    return jointLocation; //return the joint location
}

/// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
/// @param end_effector_location End effector coordinate
/// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
ManipulatorState LinkImplementation::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{

    //stre x and y values of the end effector postion
    double x = end_effector_location[0];
    double y = end_effector_location[1];
    ManipulatorState angles; //manipulator state
    ManipulatorState error_angles; //state to return in case of error (all zeros)
    error_angles.push_back(0);
    error_angles.push_back(0);

    //link lengths variables
    double a1 = 0; //for two link manipulators
    double a2;
    double a3;
    double theta1 = 0; //for two link manipulators

    //check if two or three links
    if(nLinks() == 2){
        //Three link implementation
        a2 = m_link_lengths[0];
        a3 = m_link_lengths[1];
    }else if(nLinks() == 3){
        //Three link implementation
        error_angles.push_back(0);
        a1 = m_link_lengths[0];
        a2 = m_link_lengths[1];
        a3 = m_link_lengths[2];
        theta1 = atan2(y,x); //for three link manipulator, theta 1 points the first link directly at the end effector
    }

    //subtract off the cordinates of the first joint to essentially create a two link manipulator problem for the next two joints
    //(this changes nothing for two link manipulators)
    double xp = x-a1*cos(theta1);
    double yp = y-a1*sin(theta1);

    //solve the two link manipulator problem 
    double A = ((xp*xp+yp*yp)-(a2*a2+a3*a3))/(2*a2*a3);
    double theta3;

    //make sure angle is possible
    if(A >= -1 && A <= 1){
        theta3 = -acos(A);
    }else{
        return error_angles;
    }

    double B = ((a2+a3*cos(theta3))*xp+a3*sin(theta3)*yp)/(xp*xp+yp*yp);
    double theta2;

    //make sure angle is possible
    if(B >= -1 && B <= 1){
        theta2 = acos(B) - theta1;
    }else{
        return error_angles;
    }

    //set all angles to range 0-2*pi
    if(nLinks() == 3){
        theta1 = fmod(theta1+2*M_PI,2*M_PI);
        angles.push_back(theta1);
    }

    theta2 = fmod(theta2+2*M_PI,2*M_PI);
    theta3 = fmod(theta3+2*M_PI,2*M_PI);
    angles.push_back(theta2);
    angles.push_back(theta3);

    return angles;
}

}
