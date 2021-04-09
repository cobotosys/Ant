//
// Created by cobot on 2021/3/18.
//

#include <logger/Logger.h>
#include "DualArmRobotScene.h"
#include "cobot/studio/common/CobotStudioApi.h"
#include "rw/kinematics/Kinematics.hpp"

DualArmRobotScene::DualArmRobotScene() {
    std::string ROOT_PATH = std::getenv("SDK_PATH");
    std::string filename(ROOT_PATH);
    filename.append("/install/x86-64-install/devel/data/");
    filename.append(__COBOTSYS_MODULE_NAME__);
    filename.append("/UR5DualArm.cbtx");
    LOG_INFO << "WorkCell path = " << filename;
    _workCell = cobot::studio::framework::RWWorkCellParser::parseWorkCell(filename);
    _planner = boost::make_shared<DualArmRobotPlanner>(_workCell);
    _driver = boost::make_shared<DualArmRobotDriver>(_workCell);
}

bool DualArmRobotScene::moveToFreely(rw::math::Transform3D<> left, rw::math::Transform3D<> right) {
    auto state = _planner->_state;
    auto leftQs = _planner->solverLeft(left, state, "LeftToolMount", true);
    if (leftQs.empty()) {
        LOG_INFO << "leftQs is empty";
        return false;
    } else {
        LOG_INFO << "leftQS " << leftQs.size();
        for (auto q:leftQs) {
            LOG_INFO << q;
        }
    }
    state = _planner->_state;
    auto rightQs = _planner->solverRight(right, state, "RightToolMount", true);
    if (rightQs.empty()) {
        LOG_INFO << "rightQs is empty";
        return false;
    } else {
        LOG_INFO << " rightQs " << rightQs.size();
        for (auto q:rightQs) {
            LOG_INFO << q;
        }
    }
    auto leftStartQ = _planner->_leftArmRobot->getQ(_planner->_state);
    auto rightStartQ = _planner->_rightArmRobot->getQ(_planner->_state);
    auto res = _planner->solver(leftQs, rightQs, state);
    if (res.empty()) {
        LOG_INFO << "can not find safe joints";
        return false;
    }
    auto leftGoalQ = res[0].getSubPart(0, 6);
    auto rightGoalQ = res[0].getSubPart(6, 6);
    LOG_INFO << "leftStartQ " << leftStartQ;
    LOG_INFO << "rightStartQ " << rightStartQ;
    LOG_INFO << "leftGoalQ " << leftGoalQ;
    LOG_INFO << "rightGoalQ " << rightGoalQ;
    auto checker = [&](rw::math::Q &q) {
        _planner->_c3CompositeRobot->setQ(q, _planner->_state);
        auto t = rw::kinematics::Kinematics::frameTframe(_workCell->findFrame("LeftToolMount"),
                                                         _workCell->findFrame("RightToolMount"), _planner->_state);
        auto p = t.P();
        LOG_INFO<<p.norm2();
        return !_planner->_collisionDetector->inCollision(_planner->_state) &&
               p.norm2() > 0.08; //通过距离限制两个爪子不能靠的过近，避免碰撞检测采样导致漏过碰撞姿态
//        return !_collisionDetector->inCollision(_state);
    };
    auto traj = _planner->planQPath(leftStartQ, rightStartQ, leftGoalQ, rightGoalQ, _planner->_state, checker);
    LOG_INFO << traj->duration();
    auto statePath = traj->getPath(0.05,false);
    cobot::studio::common::CobotStudioApi cobotStudioApi;
    for (auto s:statePath) {
        auto q = _planner->_c3CompositeRobot->getQ(s);
        cobotStudioApi.setQ(q.getSubPart(0, 6).toStdVector(), _planner->_leftArmRobot->getName());
        cobotStudioApi.setQ(q.getSubPart(6, 6).toStdVector(), _planner->_rightArmRobot->getName());
    }
}

bool DualArmRobotScene::moveToConstraint(rw::math::Transform3D<> left, rw::math::Transform3D<> right,
                                         boost::function<bool(rw::math::Q &)> stateConstraint) {
    auto state = _planner->_state;
    auto leftQs = _planner->solverLeft(left, state, "LeftToolMount", true);
    if (leftQs.empty()) {
        LOG_INFO << "leftQs is empty";
        return false;
    } else {
        LOG_INFO << "leftQS " << leftQs.size();
        for (auto q:leftQs) {
            LOG_INFO << q;
        }
    }
    state = _planner->_state;
    auto rightQs = _planner->solverRight(right, state, "RightToolMount", true);
    if (rightQs.empty()) {
        LOG_INFO << "rightQs is empty";
        return false;
    } else {
        LOG_INFO << " rightQs " << rightQs.size();
        for (auto q:rightQs) {
            LOG_INFO << q;
        }
    }
    auto leftStartQ = _planner->_leftArmRobot->getQ(_planner->_state);
    auto rightStartQ = _planner->_rightArmRobot->getQ(_planner->_state);
    auto res = _planner->solver(leftQs, rightQs, state);
    if (res.empty()) {
        LOG_INFO << "can not find safe joints";
        return false;
    }
    auto leftGoalQ = res[0].getSubPart(0, 6);
    auto rightGoalQ = res[0].getSubPart(6, 6);
    auto traj = _planner->planQPath(leftStartQ, rightStartQ, leftGoalQ, rightGoalQ, _planner->_state, stateConstraint);
    LOG_INFO << traj->duration();
    auto statePath = traj->getPath(0.01,false);
    cobot::studio::common::CobotStudioApi cobotStudioApi;
    for (auto s:statePath) {
        auto q = _planner->_c3CompositeRobot->getQ(s);
        cobotStudioApi.setQ(q.getSubPart(0, 6).toStdVector(), _planner->_leftArmRobot->getName());
        cobotStudioApi.setQ(q.getSubPart(6, 6).toStdVector(), _planner->_rightArmRobot->getName());
    }
}
