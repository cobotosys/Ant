//
// Created by dev on 2020/8/30.
//

#include "QConstraint.h"
#include "rw/kinematics/Kinematics.hpp"
#include "glog/logging.h"
#include "logger/Logger.h"

//Q[6]{-0.009, 0.099, 0.046, -3.09756, -0.195267, -0.460749}
bool QConstraint::doInCollision(const rw::math::Q &q) const {
    rw::kinematics::State state = _state;
    _device->setQ(q, state);
    auto tempState = state;
    if (_detector->inCollision(tempState)) {
        return true;
    } else {
        return false;
    }
//    else {
//        auto t = rw::kinematics::Kinematics::frameTframe(_workCell->findFrame("LeftPose"),
//                                                         _workCell->findFrame("RightPose"), tempState);
//        rw::math::Transform3D<> value({-0.009, 0.099, 0.046},
//                                      rw::math::RPY<>(-3.09756, -0.195267, -0.460749).toRotation3D());
//        t = (value * t);
//        LOG_INFO << t;
//        bool b = t.equal(rw::math::Transform3D<>::identity(),0.01);
//        return !b;
//    }
//    planner._c3CompositeRobot->setQ(q, tempState);
//    if (planner._collisionDetector->inCollision(tempState)) {
//        return true;
//    } else {
//        auto t = rw::kinematics::Kinematics::frameTframe(planner._workCell->findFrame("LeftPose"),
//                                                         planner._workCell->findFrame("RightPose"), tempState);
//        return !t.equal(value);
//    }
    if (_devs.size() == 2) {
        auto t = rw::kinematics::Kinematics::frameTframe(_devs[0]->getEnd(), _devs[1]->getEnd(), state);
        auto p = t.P();
//        bool b = std::abs(p(0)) < 0.05 && std::abs(p(1)) < 0.05;// && std::abs(p(2)) < 0.003;
        bool b = p.norm2() < 0.01;
        if (!b) {
            LOG(INFO) << "aa" << p(0) << "  " << p(1) << "  " << p(2);
            return true;
        }
    }
    return _detector->inCollision(state);
}

void QConstraint::doUpdate(const rw::kinematics::State &state) {
    _state = state;
}

void QConstraint::doSetLog(rw::common::Log::Ptr log) {
}

QConstraint::QConstraint() {
}
