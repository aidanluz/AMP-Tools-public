#include "AMPCore.h"

class MyClass{
    public:

        //generate a c-space obstacle given a workspace obstacle and a robot (both convex polygons)
        virtual amp::Polygon C_space_obst(amp::Polygon ob, amp::Polygon rob) const;
};