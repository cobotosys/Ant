//
// Created by cobot on 2021/3/18.
//

#include "DualArmRobotDriver.h"
#include "cobot/studio/common/DeviceFactory.h"
#include "logger/Logger.h"

DualArmRobotDriver::DualArmRobotDriver(rw::models::WorkCell::Ptr workCell) {
//    auto leftRobot = workCell->findFrame("Left_Arm");
//    auto rightRobot = workCell->findFrame("Right_Arm");
//    if (leftRobot && rightRobot) {
//        _leftClient = cobot::studio::common::DeviceFactory::createRobotApiClient(leftRobot->getPropertyMap());
//        _rightClient = cobot::studio::common::DeviceFactory::createRobotApiClient(rightRobot->getPropertyMap());
//        if (_leftClient->connect() && _rightClient->connect()) {
//
//        } else {
//            LOG_ERROR << "robot connect fail ";
//        }
//    } else {
//        LOG_FATAL << " robot not found";
//    }
}

void DualArmRobotDriver::servj(rw::math::Q q) {
    _leftClient->seroj(q.getSubPart(0, 6));
    _rightClient->seroj(q.getSubPart(6, 6));
}


rw::math::Q DualArmRobotDriver::getQ() {
    auto lq = _leftClient->getRobotJointQ();
    auto rq = _rightClient->getRobotJointQ();
    auto lvs = lq.toStdVector();
    auto rvs = rq.toStdVector();
    for (auto rv:rvs) {
        lvs.push_back(rv);
    }
    return rw::math::Q(lvs);
}
