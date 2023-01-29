#include "a3c_kinematics/Kinematics.hpp"
#include <iostream>
#include <cmath>
namespace a3c
{
    Kinematics::Kinematics() noexcept
    {
        /*
        initialize DH table in this constructor
        */
        (dhTable << 0, 0, dhParams.d1, 0,
         -M_PI_2, 0, dhParams.d2, -M_PI_2,
         0, dhParams.a2, -dhParams.d3, M_PI_2,
         M_PI_2, 0, dhParams.d4, 0,
         -M_PI_2, 0, dhParams.d5, 0,
         M_PI_2, 0, dhParams.d6, M_PI_2)
            .finished();
    }
    Pose Kinematics::fk(const JointAngles &jointAngles) noexcept
    {
        Matrix4d T = Matrix4d::Identity();
        for (size_t i = 0; i < mNumDHRows; i++)
        {
            dhTable(i, thetaIndex) += jointAngles.at(i);
            T = T * getTransformationMatrix(dhTable.row(i));
            dhTable(i, thetaIndex) -= jointAngles.at(i);
        }
        return Pose(T);
    }
    Matrix4d Kinematics::getTransformationMatrix(const Eigen::Array<double, 1, mNumDHCols> &dhRow) const noexcept
    {
        double sinTheta = sin(dhRow(thetaIndex));
        double cosTheta = cos(dhRow(thetaIndex));
        double sinAlpha = sin(dhRow(alphaIndex));
        double cosAlpha = cos(dhRow(alphaIndex));
        Matrix4d T;
        T << cosTheta, -sinTheta, 0, dhRow(aIndex),
            sinTheta * cosAlpha, cosTheta * cosAlpha, -sinAlpha, -sinAlpha * dhRow(dIndex),
            sinTheta * sinAlpha, cosTheta * sinAlpha, cosAlpha, cosAlpha * dhRow(dIndex),
            0, 0, 0, 1;
        return T;
    }
    DHParams::DHParams(float d1, float d2, float d3,
                       float d4, float d5, float d6, float a2)
        : d1(d1), d2(d2), d3(d3),
          d4(d4), d5(d5), d6(d6), a2(a2)
    {
    }
    std::ostream &operator<<(std::ostream &os, const Pose &pose)
    {
        os << "Position: \r\n"
           << "x:" << pose.position.x() << "\r\n"
           << "y:" << pose.position.y() << "\r\n"
           << "z:" << pose.position.z() << "\r\n"
           << "Orientation: \r\n"
           << "x:" << pose.orientation.x() << "\r\n"
           << "y:" << pose.orientation.y() << "\r\n"
           << "z:" << pose.orientation.z() << "\r\n"
           << "w:" << pose.orientation.w() << "\r\n\r\n";
        return os;
    }
}