#include "MyGridCSpace2DConstructor.h"
#include "MyGridCSpace2D.cpp"
#include <Eigen/Core>
#include <math.h>

using namespace std;

namespace amp{

    std::unique_ptr<amp::GridCSpace2D> MyGridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
        
        //get link lengths and obstacles
        vector<double> link_lengths = manipulator.getLinkLengths();
        vector<Polygon> obstacles = env.obstacles;

        //set c-space parameters
        size_t x0_cells = 100;
        size_t x1_cells = 100;
        double x0_min = 0;
        double x0_max = 2*M_PI;
        double x1_min = 0;
        double x1_max = 2*M_PI;

        //generate a raw C-space
        MyGridCSpace2D c_space = MyGridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max, link_lengths, obstacles);
        
        double x0;
        double x1;
        bool collision;

        //loop through each cell
        for(int i = 0; i < x0_cells; i++){
            for(int j = 0; j < x1_cells; j++){
                x0 = i*(x0_max-x0_min)/x0_cells; //generate the angle values for the cell
                x1 = j*(x1_max-x1_min)/x1_cells;
                c_space(i,j) = c_space.inCollision(x0,x1); //check if the manipulator collides with an obstacle in
            }
        }

        return make_unique<MyGridCSpace2D>(c_space); //return a pointer to the c-space
    }

}