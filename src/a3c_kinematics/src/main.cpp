#include <iostream>
#include<vector>
#include <chrono>
#include <cstdlib>
#include "a3c_kinematics/Kinematics.hpp"
using namespace std::chrono;
int main(){
    /**
     * Create kinematics object
    */
    auto kinematics = a3c::Kinematics();
    /**
     * Create test cases
    */
    std::vector<a3c::JointAngles> testCases = std::vector<a3c::JointAngles>(std::initializer_list<a3c::JointAngles>{
        {0,0,0,0,0,0},{M_PI_2,0,0,0,0,0},{0,M_PI_2,0,0,0,0},{0,0,M_PI_2,0,0,0},{0,0,0,M_PI_2,0,0}, {0,0,0,0,0,M_PI_2}
    });
    for(const auto& test:testCases){
        auto fkResult = kinematics.fk(test);
        std::cout<<fkResult;;
    }
    return 0;
}