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
    static const constexpr int no_of_iterations = 200000;
    int64_t totalTimeMicros = 0;
    for(int i=0;i<no_of_iterations;i++){
        /**
         * Creating a random number to to use the generated fk poses in a way
        */
        float randomNumber = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
        for(const auto& test:testCases){
            auto start = high_resolution_clock::now();
            auto fkResult = kinematics.fk(test);
            auto stop = high_resolution_clock::now();
            totalTimeMicros += duration_cast<microseconds>(stop - start).count();
            /**
             * Random Nonsense to ensure some action taken on calculated FK result
            */
            (fkResult.position(0)>randomNumber)?(fkResult.position(0)=(fkResult.position(2)-fkResult.position(1))):(fkResult.position)(0)+=randomNumber*2;
        }
    }
    
    
    std::cout <<"Execution Time per iteration in microseconds:"<<double(totalTimeMicros)/(no_of_iterations*testCases.size())<< std::endl;
    return 0;
}