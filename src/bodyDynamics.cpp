#include "quadruped_control_library/bodyDynamics.hpp"

#include <iostream>

BodyDynamics::BodyDynamics(std::string modelPath)
{
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(modelPath.c_str(), robotModel_.get(), true))
    {
        std::cerr << "Error loading model " << modelPath << std::endl;
        abort();
    }
RigidBodyDynamics:;
} 

void BodyDynamics::calculateDynamics()
{
    
}