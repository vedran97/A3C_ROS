#include <iostream>
#include "a3c_kinematics/ForwardKinematics.hpp"
int main(){
    auto kinematics = a3c::Kinematics();
    auto jointAngles = a3c::JointAngles{
        0,0,0,0,0,0
    };
    auto fkResult = kinematics.fk(jointAngles);
    std::cout<<" Computed FK, returning"<<std::endl;
    return 0;
}