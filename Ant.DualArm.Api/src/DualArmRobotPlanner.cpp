//
// Created by cobot on 2021/3/11.
//
#include "StatePlaner.h"
#include <rw/pathplanning/QEdgeConstraint.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include "../include/DualArmRobotPlanner.h"
#include "rwlibs/proximitystrategies/ProximityStrategyFactory.hpp"
#include "logger/Logger.h"
#include "rwlibs/pathplanners/rrt/RRTPlanner.hpp"
#include "rw/pathplanning/QSampler.hpp"
#include "rw/pathplanning/PlannerUtil.hpp"
#include "rw/invkin/ClosedFormIKSolverUR.hpp"
#include "rw/invkin/JacobianIKSolver.hpp"
#include "rw/kinematics/Kinematics.hpp"
#include "cobot/studio/common/CobotStudioApi.h"

DualArmRobotPlanner::DualArmRobotPlanner(rw::models::WorkCell::Ptr workCell, std::string leftArm, std::string rightArm) : _workCell(
        workCell), _leftArmName(leftArm),
                                                                                                                          _rightArmName(
                                                                                                              rightArm) {
    auto strategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy();
    _collisionDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_workCell, strategy));
    _state = _workCell->getDefaultState();
    _leftArmRobot = _workCell->findDevice(_leftArmName);
    if (!_leftArmRobot) {
        LOG_FATAL << "empty left arm " << _leftArmName;
    }
    _rightArmRobot = _workCell->findDevice(_rightArmName);
    if (!_rightArmRobot) {
        LOG_FATAL << "empty right arm " << _rightArmName;
    }
    std::vector<rw::models::Device::Ptr> devices{_leftArmRobot, _rightArmRobot};
    std::vector<rw::kinematics::Frame *> ends{_leftArmRobot->getEnd(), _rightArmRobot->getEnd()};
    _c3CompositeRobot = rw::common::ownedPtr(
            new rw::models::CompositeDevice(_workCell->getWorldFrame(), devices, ends, "C3", _state));
}

#include "QConstraint.h"
#include "rw/trajectory/TimedUtil.hpp"
#include "rw/trajectory/TrajectoryFactory.hpp"

rw::common::Ptr<rw::trajectory::Trajectory<rw::kinematics::State>>
DualArmRobotPlanner::planQPath(rw::math::Q leftStart, rw::math::Q rightStart, rw::math::Q leftGoal, rw::math::Q rightGoal,
                               rw::kinematics::State &state, boost::function<bool(rw::math::Q &)> stateChecker) {
    auto start = combineQ(leftStart, rightStart);
    auto end = combineQ(leftGoal, rightGoal);
    auto box = _c3CompositeRobot->getBounds();
    StatePlaner statePlaner(12);
    std::vector<double> values;
    for (int i = 0; i < 24; i++) {
        if (i % 2 == 0) {
            values.push_back(-3.14);
        } else {
            values.push_back(3.14);
        }
    }
    statePlaner.setBounds({values});
    statePlaner.setStateChecker(stateChecker);
//    statePlaner.setStateChecker([&](rw::math::Q &q) {
//        _c3CompositeRobot->setQ(q, _state);
//        auto t = rw::kinematics::Kinematics::frameTframe(_workCell->findFrame("LeftToolMount"),
//                                                         _workCell->findFrame("RightToolMount"), _state);
//        auto p = t.P();
//        return !_collisionDetector->inCollision(_state) && p.norm2() > 0.1; //通过距离限制两个爪子不能靠的过近，避免碰撞检测采样导致漏过碰撞姿态
////        return !_collisionDetector->inCollision(_state);
//    });
    auto path = statePlaner.solve(20, start, end, 0.03);
    cobot::studio::common::CobotStudioApi cobotStudioApi;
//    for (auto q:path) {
//        _c3CompositeRobot->setQ(q, _state);
//        cobotStudioApi.setQ(q.getSubPart(0, 6).toStdVector(), _leftArmRobot->getName());
//        cobotStudioApi.setQ(q.getSubPart(6, 6).toStdVector(), _rightArmRobot->getName());
//    }
//    std::exit(11);
    auto timedPath = rw::trajectory::TimedUtil::makeTimedStatePath(
            *_workCell,
            rw::models::Models::getStatePath(*_c3CompositeRobot, path, state));

    auto traj = rw::trajectory::TrajectoryFactory::makeLinearTrajectory(timedPath);
    return traj;
//    if (enableCons) {
//        rw::common::Ptr<QConstraint> myconstraint = rw::common::ownedPtr(
//                new QConstraint());
//        myconstraint->_detector = _collisionDetector;
//        myconstraint->_device = _c3CompositeRobot;
//        myconstraint->_devs.push_back(_leftArmRobot);
//        myconstraint->_devs.push_back(_rightArmRobot);
//        myconstraint->_state = state;
//        myconstraint->_workCell = _workCell;
//        rw::pathplanning::QEdgeConstraint::Ptr edge =
//                rw::pathplanning::QEdgeConstraint::makeDefault(myconstraint, _c3CompositeRobot);
//
//        const rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(myconstraint,
//                                                                                                         edge);
//        auto type = rwlibs::pathplanners::RRTPlanner::RRTBalancedBidirectional;
//        auto sample = rw::pathplanning::QSampler::makeUniform(_c3CompositeRobot);
//        auto metric = rw::pathplanning::PlannerUtil::normalizingInfinityMetric(_c3CompositeRobot->getBounds());
//        auto planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sample, metric, 0.001, type);
////        auto planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, _c3CompositeRobot);
//        rw::trajectory::QPath path;
//        planner->query(start, end, path);
//        auto timedPath = rw::trajectory::TimedUtil::makeTimedStatePath(
//                *_workCell,
//                rw::models::Models::getStatePath(*_c3CompositeRobot, path, state));
//
//        auto traj = rw::trajectory::TrajectoryFactory::makeLinearTrajectory(timedPath);
//        return traj;
//    } else {
//        rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(_collisionDetector,
//                                                                                                   _c3CompositeRobot,
//                                                                                                   state);
//        auto planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, _c3CompositeRobot);
//        rw::trajectory::QPath path;
//        planner->query(start, end, path);
//        auto timedPath = rw::trajectory::TimedUtil::makeTimedStatePath(
//                *_workCell,
//                rw::models::Models::getStatePath(*_c3CompositeRobot, path, state));
//
//        auto traj = rw::trajectory::TrajectoryFactory::makeLinearTrajectory(timedPath);
//        return traj;
//    }
}

rw::math::Q DualArmRobotPlanner::combineQ(rw::math::Q q1, rw::math::Q q2) {
    std::vector<double> values;
    auto sqv1 = q1.toStdVector();
    auto sqv2 = q2.toStdVector();
    values.insert(values.end(), sqv1.begin(), sqv1.end());
    values.insert(values.end(), sqv2.begin(), sqv2.end());
    return {values};
}

std::vector<rw::math::Q>
DualArmRobotPlanner::solverLeft(rw::math::Transform3D<> t, rw::kinematics::State &state, std::string tool, bool isUR) {
    std::vector<rw::math::Q> qs;
    auto toolMount = _workCell->findFrame(tool);
    if (!toolMount) {
        LOG_FATAL << " empty tool " << tool;
    }
    if (isUR) {
        rw::models::SerialDevice::Ptr serialDevice = _leftArmRobot.cast<rw::models::SerialDevice>();
        rw::invkin::ClosedFormIKSolverUR leftSolver(serialDevice, state);
        auto toolToTCP = rw::kinematics::Kinematics::frameTframe(toolMount, _leftArmRobot->getEnd(), state);
        LOG_INFO << t;
        LOG_INFO << toolToTCP;
        LOG_INFO << t * toolToTCP;
        qs = leftSolver.solve(t * toolToTCP, state);
    } else {
        rw::invkin::JacobianIKSolver leftSolver(_leftArmRobot, toolMount, state);
        qs = leftSolver.solve(t, state);
    }
//    if (qs.empty()) {
//        LOG_WARNING << " can not  Left solver " << t;
//        return qs;
//    } else {
//        std::vector<rw::math::Q> res;
//        for (auto q:qs) {
//            _leftArmRobot->setQ(q, state);
//            if (!_collisionDetector->inCollision(state)) {
//                res.push_back(q);
//            } else {
//                LOG_INFO << " inCollision q " << q;
//            }
//        }
//        return res;
//    }
    return qs;
}

std::vector<rw::math::Q>
DualArmRobotPlanner::solverRight(rw::math::Transform3D<> t, rw::kinematics::State &state, std::string tool, bool isUR) {
    std::vector<rw::math::Q> qs;
    auto toolMount = _workCell->findFrame(tool);
    if (!toolMount) {
        LOG_FATAL << " empty tool " << tool;
    }
    if (isUR) {
        rw::models::SerialDevice::Ptr serialDevice = _rightArmRobot.cast<rw::models::SerialDevice>();
        rw::invkin::ClosedFormIKSolverUR leftSolver(serialDevice, state);
        auto toolToTCP = rw::kinematics::Kinematics::frameTframe(toolMount, _rightArmRobot->getEnd(), state);
        qs = leftSolver.solve(t * toolToTCP, state);
    } else {
        rw::invkin::JacobianIKSolver leftSolver(_rightArmRobot, toolMount, state);
        qs = leftSolver.solve(t, state);
    }
//    if (qs.empty()) {
//        LOG_WARNING << " can not Right solver " << t;
//        return qs;
//    } else {
//        std::vector<rw::math::Q> res;
//        for (auto q:qs) {
//            _rightArmRobot->setQ(q, state);
//            if (!_collisionDetector->inCollision(state)) {
//                res.push_back(q);
//            }
//        }
//        return res;
//    }
    return qs;
}

std::vector<rw::math::Q> DualArmRobotPlanner::solver(std::vector<rw::math::Q> leftQs, std::vector<rw::math::Q> rightQS,
                                                     rw::kinematics::State state) {
    std::vector<rw::math::Q> res;
    for (auto lq:leftQs) {
        for (auto rq:rightQS) {
            auto q = combineQ(lq, rq);
            _c3CompositeRobot->setQ(q, state);
            if (!_collisionDetector->inCollision(state)) {
                res.push_back(q);
            }
        }
    }
    return res;
}
