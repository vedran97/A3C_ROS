#include <iostream>
#include<vector>
#include "a3c_kinematics/ForwardKinematics.hpp"
int main(){
    auto kinematics = a3c::Kinematics();

    const std::vector<a3c::JointAngles> testCases = std::vector<a3c::JointAngles>(std::initializer_list<a3c::JointAngles>{
        {0,0,0,0,0,0},{M_PI_2,0,0,0,0,0},{0,M_PI_2,0,0,0,0},{0,0,M_PI_2,0,0,0},{0,0,0,M_PI_2,0,0}, {0,0,0,0,0,M_PI_2}
    });
    for(const auto& test:testCases){
        std::cout<<"position:\r\n"<<kinematics.fk(test).position<<std::endl;
    }
    return 0;
}