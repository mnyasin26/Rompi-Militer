//
// Created by yangcheng on 2018/11/27.
//

#include <cmath>
// #include "../include/eigen3/Eigen/Dense"
#include <ArduinoEigenDense.h>
#include "Quaternions.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
using namespace Eigen;

Quaternions::Quaternions() {};

Quaternions::~Quaternions() = default;

// 归一化.
Vector4d Quaternions::Normalise(Vector4d &q) const {
    Vector4d normQ;
    double norm2 = q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
    // 如果四元数各项足够接近单位四元数, 则不做任何处理.
    if (norm2 != 0.0) {
        double norm = sqrt(norm2);
        normQ(0) = q(0) / norm;
        normQ(1) = q(1) / norm;
        normQ(2) = q(2) / norm;
        normQ(3) = q(3) / norm;
    } else {
        normQ = q;
    }
    return normQ;
}

// 共轭四元数, 实部相同，虚部取反.
Vector4d Quaternions::GetConjugate(Vector4d &q) const {
    Vector4d conjQ;
    conjQ(0) = q(0);
    conjQ(1) = -q(1);
    conjQ(2) = -q(2);
    conjQ(3) = -q(3);
    return conjQ;
}

// 四元数基本运算, 加.
Vector4d Quaternions::Add(Vector4d &q1, Vector4d &q2) const {
    Vector4d addRes;

    addRes(0) = q1(0) + q2(0);
    addRes(1) = q1(1) + q2(1);
    addRes(2) = q1(2) + q2(2);
    addRes(3) = q1(3) + q2(3);

    return addRes;
}

// 四元数基本运算, 点乘.
Vector4d Quaternions::DotMulti(Vector4d &q1, Vector4d &q2) const {
    Vector4d dotMultiRes;

    dotMultiRes(0) = q1(0) * q2(0);
    dotMultiRes(1) = q1(1) * q2(1);
    dotMultiRes(2) = q1(2) * q2(2);
    dotMultiRes(3) = q1(3) * q2(3);

    return dotMultiRes;
}

// 四元数基本运算, 叉乘.
Vector4d Quaternions::CrossMulti(Vector4d &q1, Vector4d &q2) const {
    Vector4d crossMultiRes;

    crossMultiRes(0) = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    crossMultiRes(1) = q1(1) * q2(0) + q1(0) * q2(1) + q1(2) * q2(3) - q1(3) * q2(2);
    crossMultiRes(2) = q1(2) * q2(0) + q1(0) * q2(2) + q1(3) * q2(1) - q1(1) * q2(3);
    crossMultiRes(3) = q1(3) * q2(0) + q1(0) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1);

    return crossMultiRes;
}

// 从欧拉角 v(x, y, z)/v(Roll, Pitch, Yaw) 中获取四元数 Q(q0,q1,q2,q3).
Vector4d Quaternions::GetQFromEuler(Vector3d &euler_angle) const {
    Vector4d eulerQ;

    double r = (euler_angle(0) * M_PI / 180.0) / 2.0;
    double p = (euler_angle(1) * M_PI / 180.0) / 2.0;
    double y = (euler_angle(2) * M_PI / 180.0) / 2.0;


    double sinp = sin(p);
    double siny = sin(y);
    double sinr = sin(r);
    double cosp = cos(p);
    double cosy = cos(y);
    double cosr = cos(r);

    eulerQ(0) = cosr * cosp * cosy + sinr * sinp * siny;
    eulerQ(1) = sinr * cosp * cosy - cosr * sinp * siny;
    eulerQ(2) = cosr * sinp * cosy + sinr * cosp * siny;
    eulerQ(3) = cosr * cosp * siny - sinr * sinp * cosy;

    return eulerQ;
}

// 从四元数获取余弦矩阵DCM
Matrix3d Quaternions::GetDCMFromQ(Vector4d &q) {
    // b系到地理系n系
    Matrix3d dcm_b2n;

    dcm_b2n(0, 0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
    dcm_b2n(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
    dcm_b2n(0, 2) = 2 * (q(1) * q(3) + q(0) * q(2));
    dcm_b2n(1, 0) = 2 * (q(1) * q(2) + q(0) * q(3));
    dcm_b2n(1, 1) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
    dcm_b2n(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));
    dcm_b2n(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
    dcm_b2n(2, 1) = 2 * (q(2) * q(3) + q(0) * q(1));
    dcm_b2n(2, 2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

    return dcm_b2n;
}

// 从余弦矩阵DCM获取四元数
Vector4d Quaternions::GetQfromDCM(Matrix3d &dcm_b2n) {

    Vector4d q;
    double trace = dcm_b2n(0,0) + dcm_b2n(1,1) + dcm_b2n(2,2); // I removed + 1.0f; see discussion with Ethan
    if( trace > 0 ) {// I changed M_EPSILON to 0
        double s = 0.5 / sqrt(trace+ 1.0);
        q(0) = 0.25 / s;
        q(1) = ( dcm_b2n(2,1) - dcm_b2n(1,2) ) * s;
        q(2) = ( dcm_b2n(0,2) - dcm_b2n(2,0) ) * s;
        q(3) = ( dcm_b2n(1,0) - dcm_b2n(0,1) ) * s;
    } else {
        if ( dcm_b2n(0,0) > dcm_b2n(1,1) && dcm_b2n(0,0) > dcm_b2n(2,2) ) {
            double s = 2.0 * sqrt( 1.0 + dcm_b2n(0,0) - dcm_b2n(1,1) - dcm_b2n(2,2));
            q(0) = (dcm_b2n(2,1) - dcm_b2n(1,2) ) / s;
            q(1) = 0.25 * s;
            q(2) = (dcm_b2n(0,1) + dcm_b2n(1,0) ) / s;
            q(3) = (dcm_b2n(0,2) + dcm_b2n(2,0) ) / s;
        } else if (dcm_b2n(1,1) > dcm_b2n(2,2)) {
            double s = 2.0 * sqrt( 1.0 + dcm_b2n(1,1) - dcm_b2n(0,0) - dcm_b2n(2,2));
            q(0) = (dcm_b2n(0,2) - dcm_b2n(2,0) ) / s;
            q(1) = (dcm_b2n(0,1) + dcm_b2n(1,0) ) / s;
            q(2) = 0.25 * s;
            q(3) = (dcm_b2n(1,2) + dcm_b2n(2,1)) / s;
        } else {
            double s = 2.0 * sqrt( 1.0 + dcm_b2n(2,2) - dcm_b2n(0,0) - dcm_b2n(1,1) );
            q(0) = (dcm_b2n(1,0) - dcm_b2n(0,1) ) / s;
            q(1) = (dcm_b2n(0,2) + dcm_b2n(2,0) ) / s;
            q(2) = (dcm_b2n(1,2) + dcm_b2n(2,1) ) / s;
            q(3) = 0.25 * s;
        }
    }

//    q(0) = 0.5 * sqrt(1.0 + dcm_b2n(0, 0) + dcm_b2n(1, 1) + dcm_b2n(2, 2));
//    double beta = 1.0 / (4.0 * q(0));
//    q(1) = beta * (dcm_b2n(2,1) - dcm_b2n(1,2));
//    q(2) = beta * (dcm_b2n(0,2) - dcm_b2n(2,0));
//    q(3) = beta * (dcm_b2n(1,0) - dcm_b2n(0,1));
    return q;
}

Vector3d Quaternions::GetEulerFromQ(Vector4d &q) {
    Vector3d euler;

    euler(0) = atan2(2.0 * (q(0) * q(1) + q(2) * q(3)), 1 - 2 * (q(1) * q(1) + q(2) * q(2)));
    euler(1) = asin(2 * (q(0) * q(2) - q(3) * q(1)));
    euler(2) = atan2(2 * (q(0) * q(3) + q(1) * q(2)), 1 - 2 * (q(2) * q(2) + q(3) * q(3)));
    return euler;
}