#include "a3c_kinematics/Kinematics.hpp"
#include<iostream>
#include<cmath>
namespace a3c{
Kinematics::Kinematics(){
/*
initialize DH table in this constructor
*/
(dhTable << 0, 0, 0.1915, 0, 
          -M_PI_2, 0, 0.1405, -M_PI_2,
          0, 0.230, -0.1415, M_PI_2,
          M_PI_2, 0, 0.230, 0,
          -M_PI_2, 0, 0.1635, 0,
          M_PI_2, 0, 0.1665,M_PI_2).finished();

}
Pose Kinematics::fk(const JointAngles& jointAngles) {
    Matrix4d T = Matrix4d::Identity();
    for (int i=0; i<mNumDHRows; i++){
        dhTable(i,thetaIndex) += jointAngles.at(i);
        T = T* getTransformationMatrix(dhTable.row(i));
        dhTable(i,thetaIndex) -= jointAngles.at(i);

    }
    auto retPose = Pose();
    retPose.position = {T(0,3),T(1,3),T(2,3)};
    retPose.orientation = Eigen::Quaterniond(T.topLeftCorner<3, 3>());
    return retPose;
}
Matrix4d Kinematics::getTransformationMatrix(const Eigen::Array<double,1,mNumDHCols>& dhRow) const{
    double sinTheta = sin(dhRow(thetaIndex));
    double cosTheta = cos(dhRow(thetaIndex));
    double sinAlpha = sin(dhRow(alphaIndex));
    double cosAlpha = cos(dhRow(alphaIndex));
    Matrix4d T;
    T <<cosTheta, -sinTheta, 0, dhRow(aIndex),
        sinTheta*cosAlpha, cosTheta*cosAlpha, -sinAlpha, -sinAlpha*dhRow(dIndex),
        sinTheta*sinAlpha, cosTheta*sinAlpha, cosAlpha, cosAlpha*dhRow(dIndex),
        0,0,0,1;
    return T;
}
  
}