#ifndef A3C_FK_HPP
#define A3C_FK_HPP
#include <eigen3/Eigen/Dense>
#include <array>
#include <iostream>

using namespace Eigen;

namespace a3c
{
    using JointAngles = std::array<double, 6>;
    struct Pose
    {
        Pose(const Matrix4d &T)
        {
            this->position = {T(0, 3), T(1, 3), T(2, 3)};
            this->orientation = Eigen::Quaterniond(T.topLeftCorner<3, 3>());
        }
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        friend std::ostream &operator<<(std::ostream &out, const Pose &pose);
    };
    struct DHParams
    {
        float d1, d2, d3, d4, d5, d6, a2; // They are expected to be in metres
        DHParams(float d1, float d2, float d3,
                 float d4, float d5, float d6, float a2);
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
        /**
         * Declaring these for the A3C here, later can be read from a file in the constructor and set
         */
        const DHParams dhParams{0.1915, 0.1405, 0.1415, 0.230, 0.1635, 0.1665, 0.230};

    public:
        Pose fk(const JointAngles &ja) noexcept;
        Kinematics() noexcept;
        Matrix4d getTransformationMatrix(const Eigen::Array<double, 1, mNumDHCols> &dhRow) const noexcept;
    };
}

#endif
