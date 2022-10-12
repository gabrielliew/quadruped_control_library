#ifndef SRC_QUADRUPED_CONTROL_INCLUDE_QUADRUPED_CONTROL_LIBRARY_BODYDYNAMICS_HPP
#define SRC_QUADRUPED_CONTROL_INCLUDE_QUADRUPED_CONTROL_LIBRARY_BODYDYNAMICS_HPP

#include <Eigen/Core>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <string>
#include <memory>

class BodyDynamics
{
public:
    BodyDynamics(std::string modelPath);
    void calculateDynamics();

private:
    std::unique_ptr<RigidBodyDynamics::Model> robotModel_;
};

#endif /* SRC_QUADRUPED_CONTROL_INCLUDE_QUADRUPED_CONTROL_LIBRARY_BODYDYNAMICS_HPP */
