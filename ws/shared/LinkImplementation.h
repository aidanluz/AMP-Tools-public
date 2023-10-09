#include "tools/LinkManipulator.h"

namespace amp {

class LinkImplementation : public LinkManipulator2D {
    public:

        /******* User Implemented Methods ********/

        LinkImplementation() : LinkManipulator2D(){}

        /// @brief Construct from an array of link lengths. The base location is set to (0.0, 0.0)
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        LinkImplementation(const std::vector<double>& link_lengths) : LinkManipulator2D(link_lengths){}

        /// @brief Construct from an array of link lengths.
        /// @param base_location Custom base location of the manipulator
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        LinkImplementation(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths) : LinkManipulator2D(base_location,link_lengths){}

        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;

        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        ~LinkImplementation(){}
};


}