#ifndef A3C_FK_HPP
#define A3C_FK_HPP
#include <eigen3/Eigen/Dense>
#include <array>
using namespace Eigen;
namespace a3c{
using JointAngles = std::array<double, 6>;
struct Pose {
    Pose(const Matrix4d& T){
        this->position = {T(0,3),T(1,3),T(2,3)};
        this->orientation = Eigen::Quaterniond(T.topLeftCorner<3, 3>());
    }
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
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
