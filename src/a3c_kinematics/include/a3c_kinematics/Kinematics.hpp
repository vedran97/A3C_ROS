#ifndef A3C_FK_HPP
#define A3C_FK_HPP
#include <eigen3/Eigen/Dense>
#include <array>
using namespace Eigen;
using Eigen::MatrixXd;
namespace a3c{
using JointAngles = std::array<double, 6>;
struct Pose {
    Eigen::Vector3d position;
};
class Kinematics
{
    private: 
        constexpr static const size_t mNumDHRows = 6;
        constexpr static const size_t mNumDHCols = 4;
        constexpr static const size_t alphaIndex = 0;
        constexpr static const size_t aIndex = 1;
        constexpr static const size_t dIndex = 2;
        constexpr static const size_t thetaIndex = 3;
        using DHTable = Eigen::Array<double, mNumDHRows, mNumDHCols>;
        DHTable dhTable;
    public:
        Pose fk(const JointAngles& ja) ;
        Kinematics();
        Matrix4d getTransformationMatrix(const Eigen::Array<double,1,mNumDHCols>& dhRow) const;
};
}

#endif
