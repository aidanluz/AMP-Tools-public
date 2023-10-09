#include "tools/ConfigurationSpace.h"
#include "AMPCore.h"
#include "hw/HW4.h"

namespace amp {

/// @brief User implemented abstract class that accesses the continuous C-Space (bounded)
class MyGridCSpace2D : public GridCSpace2D {
    public:

        //constructors
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max) : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){}

        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max, const std::vector<double>& link_lengths, const std::vector<Polygon> obstacles) : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){
            c_link_lengths = link_lengths;
            c_obstacles = obstacles;
        }
        
        /******* User Implemented Methods ********/

        /// @brief Access the C-space with continuous variables (interpolation between cells)
        /// @param x0 Value of the first configuration variable
        /// @param x1 Value of the second configuration variable
        /// @return `true` if the the point is in collision, `false` if it is not
        bool inCollision(double x0, double x1) const override;

        const std::vector<double>& getLinkLengths() const {return c_link_lengths;}

        const std::vector<Polygon>& getObstacles() const {return c_obstacles;}
        /*****************************************/

        ~MyGridCSpace2D() {}
    protected:
        std::vector<Polygon> c_obstacles;
        std::vector<double> c_link_lengths;
};

}