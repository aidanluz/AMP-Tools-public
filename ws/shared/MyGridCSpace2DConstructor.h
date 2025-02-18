#include "hw/HW4.h"

namespace amp {

class MyGridCSpace2DConstructor : public GridCSpace2DConstructor {
    public:

        /// @brief Construct a CSpace from a manipulator and an environment
        /// @param manipulator Two link manipulator (consider ussing `ASSERT` to make sure the manipulator is 2D)
        /// @param env Environment
        /// @return Unique pointer to your C-space. 
        /// NOTE: We use a unique pointer here to be able to move the C-space without copying it, since grid discretization
        /// C-spaces can contain a LOT of memory, so copying would be a very expensive operation. Additionally, a pointer is polymorphic
        /// which allows the type to pose as a GridCSpace2D (even though GridCSpace2D is abstract)
        std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
    
        ~MyGridCSpace2DConstructor() {}
};

}