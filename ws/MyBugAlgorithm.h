#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) const override;

        // Add any other methods here...
        bool detect_collision(std::vector<amp::Polygon> obstacles, Eigen::Vector2d& curr_pos, double prevx, double prevy, bool on_object, int curr_int_ob_num, int& int_ob_num, int& int_edge_num) const;
    
    private:
        // Add any member variables here...
};