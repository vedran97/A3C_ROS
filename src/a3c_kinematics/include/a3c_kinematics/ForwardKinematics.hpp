#ifndef A3C_FK_HPP
#define JOGGER_BLENDING_HPP
#include <eigen3/Eigen/Dense>
#include <array>
using namespace Eigen;
namespace a3c{
using JointAngles = std::array<double, 6>;
struct Pose {
    Eigen::Vector3d position;
};
class Kinematics
{
    private: 
        constexpr static const size_t mNumDHRows = 6;
        using DHTable = Eigen::Array<double, mNumDHRows, 4>;
        DHTable dHTable;
    public:
        Pose fk(const JointAngles& ja) const;
        Kinematics();
};
}

#endif
