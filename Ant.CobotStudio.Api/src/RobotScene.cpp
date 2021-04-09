/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 2021-4-9          xuzhenhai
============================================================== **/

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <StatePlaner.h>
#include "RobotScene.h"
#include "RWWorkCellParser.h"
#include "logger/Logger.h"
#include "rw/invkin/JacobianIKSolver.hpp"

RobotScene::RobotScene(const std::string workCellPath) : _workCellPath(workCellPath) {
    _workCell = cobot::studio::framework::RWWorkCellParser::parseWorkCell(workCellPath);
    auto strategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy();
    _collisionDetector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_workCell, strategy));
    _state = _workCell->getDefaultState();
    initDevice();
}

RobotScene::~RobotScene() {
//    modbus_free(_ctx);
}

void RobotScene::initDevice() {
    auto devices = _workCell->getDevices();
    for (auto device:devices) {
        auto &propertyMap = device->getPropertyMap();
        //从场景文件中获取机器人的配置信息，创建机器人对象
        if (propertyMap.has("ApiType") && propertyMap.get<std::string>("ApiType") == "Robot") {
            _robotDevice = device;
            _robotClient = DeviceFactory::createRobotApiClient(propertyMap);
            try {
                if (!_robotClient->connect()) {
                    LOG_ERROR << "cobot connect fail please check the network ";
                }
            } catch (...) {
                LOG_ERROR << "cobot connect fail please check the network ";
            }
        }
    }
    auto frames = _workCell->getFrames();
    for (auto f:frames) {
        auto &propertyMap = f->getPropertyMap();
        if (propertyMap.has("Tool") && propertyMap.get<std::string>("Tool") == "true") {
            if (propertyMap.has("url")) {
                std::string &portName = propertyMap.get<std::string>("url");
                _ctx = modbus_new_rtu(portName.c_str(), 9600, 'N', 8, 1);
                modbus_set_debug(_ctx, TRUE);
                modbus_set_slave(_ctx, 0xA0);  // 设置从机地址
                // connect
                if (-1 == modbus_connect(_ctx)) {
                    LOG_ERROR << " grisp tool connect fail " << portName;
                    modbus_free(_ctx);
                } else {
                    LOG_INFO << " grisp tool connect sucess";
                }
            }
        } else if (propertyMap.has("ApiType") && propertyMap.get<std::string>("ApiType") == "Camera3D") {
            _camera3DClient = DeviceFactory::create3DCameraApiClient(propertyMap);
        }
    }
}

void RobotScene::openWorkCellInCobotStudio() {
    _cobotStudioApi.openWorkCell(_workCellPath);
}

rw::trajectory::QPath RobotScene::query(rw::math::Q start, rw::math::Q goal, double time, double dis) {
    int dof = _robotDevice->getDOF();
    StatePlaner statePlaner(dof);
    std::vector<double> values;
    auto bounds = _robotDevice->getBounds();
    //设置每一个关节的上下限
    for (int i = 0; i < dof; i++) {
        values.push_back(bounds.first[i]);
        values.push_back(bounds.second[i]);
    }
    //目前的状态检测只是使用了碰撞检测
    auto stateCheck = [&](rw::math::Q &q) {
        _robotDevice->setQ(q, _state);
        return !_collisionDetector->inCollision(_state);
    };
    statePlaner.setBounds({values});
    statePlaner.setStateChecker(stateCheck);
    auto path = statePlaner.solve(time, start, goal, dis);
    return path;
}

rw::trajectory::StateTrajectory::Ptr RobotScene::makeLinearTrajectory(rw::trajectory::QPath &qpath) {
    auto timedPath = rw::trajectory::TimedUtil::makeTimedStatePath(
            *_workCell,
            rw::models::Models::getStatePath(*_robotDevice, qpath, _state));
    auto traj = rw::trajectory::TrajectoryFactory::makeLinearTrajectory(timedPath);
    return traj;
}

int RobotScene::openHand() {
    return modbus_write_register(_ctx, 999, 0); //open
}

int RobotScene::closeHand() {
    return modbus_write_register(_ctx, 999, 200); //close
}

bool RobotScene::executeTrajectory(rw::trajectory::StateTrajectory::Ptr traj, double timeSample, bool sim) {
    auto states = traj->getPath(timeSample, false);
    for (auto state:states) {
        usleep(1000 * 1000 * timeSample);
        auto q = _robotDevice->getQ(state);
        if (!_collisionDetector->inCollision(state)) {
            _cobotStudioApi.setQ(q.toStdVector(), _robotDevice->getName());
            if (!sim) {
                _robotClient->seroj(q);
            }
        } else {
            LOG_ERROR << " joint position is inCollision " << q;
            return false;
        }
    }
    return true;
}

CobotStudioApi RobotScene::getCobotStudioApi() {
    return _cobotStudioApi;
}

rw::proximity::CollisionDetector::Ptr RobotScene::getCollisionDetector() {
    return _collisionDetector;
}

rw::kinematics::State RobotScene::getState() {
    return _state;
}

rw::models::Device::Ptr RobotScene::getRobotDevice() {
    return _robotDevice;
}

std::shared_ptr<Falcon::Camera::Common::Camera3DClient> RobotScene::getCamera3DClient() {
    return _camera3DClient;
}

std::shared_ptr<Sensor::RobotDriver::RobotClient> RobotScene::getRobotClient() {
    return _robotClient;
}

std::vector<rw::math::Q> RobotScene::solver(rw::math::Transform3D<> transform3D) {
    rw::invkin::JacobianIKSolver jacobianIkSolver(_robotDevice, _workCell->findFrame("graspFrame"), _state);
    auto qs = jacobianIkSolver.solve(transform3D, _state);
    std::vector<rw::math::Q> result;
    for (auto q:qs) {
        _robotDevice->setQ(q, _state);
        if (!_collisionDetector->inCollision(_state)) {
            result.push_back(q);
        }
    }
    return result;
}
