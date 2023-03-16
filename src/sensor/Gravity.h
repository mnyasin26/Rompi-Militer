//
// Created by yangcheng on 2019/4/13.
//

#ifndef LOCATION_GRAVITY_H
#define LOCATION_GRAVITY_H

// #include "../include/eigen3/Eigen/Dense"
#include <ArduinoEigenDense.h>
#include "../system/Status.h"

class Gravity {

public:

    bool IsShaking(routing::Status *status, Eigen::Vector3d &g_data);
};


#endif //LOCATION_GRAVITY_H
