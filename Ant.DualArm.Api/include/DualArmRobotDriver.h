//
// Created by cobot on 2021/3/18.
//

#ifndef SPARROWC3TEST_DUALARMROBOTDRIVER_H
#define SPARROWC3TEST_DUALARMROBOTDRIVER_H


#include <rw/math/Q.hpp>
#include "RobotClient.h"
#include "rw/models/WorkCell.hpp"

class DualArmRobotDriver {
public:
    DualArmRobotDriver(rw::models::WorkCell::Ptr workCell);


public:
    void servj(rw::math::Q q);

    rw::math::Q getQ();

private:
    std::shared_ptr<Sensor::RobotDriver::RobotClient> _leftClient;
    std::shared_ptr<Sensor::RobotDriver::RobotClient> _rightClient;
    rw::models::WorkCell::Ptr _workCell;
};


#endif //SPARROWC3TEST_DUALARMROBOTDRIVER_H
