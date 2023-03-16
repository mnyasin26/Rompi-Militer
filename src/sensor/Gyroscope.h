//
// Created by yangcheng on 2018/12/13.
//

// #include "../include/eigen3/Eigen/Dense"
#include <ArduinoEigenDense.h>
#include "../system/Status.h"

#ifndef LOCATION_GYROSCOPE_H
#define LOCATION_GYROSCOPE_H


class Gyroscope {
public:

    // 从陀螺仪获取姿态旋转矩阵(方向余弦矩阵DCM), b系坐标转g系
//    Matrix3d GetDCM(Vector3d &gyro, double &deltaT);

    // 陀螺仪标定
    void GyroCalibration(Eigen::MatrixXd &input_data, routing::Status *status);

};


#endif //LOCATION_GYROSCOPE_H
