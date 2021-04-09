/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 2021-4-9          xuzhenhai
============================================================== **/

#ifndef ANTCOBOTSTUDIOAPI_ROBOTSCENE_H
#define ANTCOBOTSTUDIOAPI_ROBOTSCENE_H


#include <string>
#include <rw/proximity/CollisionDetector.hpp>
#include "cobot/studio/common/CobotStudioApi.h"
#include "rw/models.hpp"
#include "cobot/studio/common/DeviceFactory.h"
#include "modbus/modbus.h"
#include "logger/Logger.h"

using namespace cobot::studio::common;

/**
 * 机器人运动场景
 */
class RobotScene {
public:
    /**
     *  构建场景
     * @param workCellPath  场景文件cbts文件的路径
     */
    RobotScene(const std::string workCellPath);

    /**
     * 析构函数
     */
    virtual ~RobotScene();

    /**
     * 在CobotStudio中打开当前的场景文件，注意请先手动启动CobotStudio
     */
    void openWorkCellInCobotStudio();

    /**
     * 使用RRT查询路径
     * @param start  开始的机器人关节姿态
     * @param goal   结束的机器人关节姿态
     * @param time    规划的时间
     * @param dis    rrt采样距离
     * @return
     */
    rw::trajectory::QPath query(rw::math::Q start, rw::math::Q goal, double time = 20, double dis = 0.03);

    /**
     * 对路经进行线性差不
     * @param qpath  机器人运动路径
     * @return   机器人轨迹
     */
    rw::trajectory::StateTrajectory::Ptr makeLinearTrajectory(rw::trajectory::QPath &qpath);

    /**
     * 执行机器人轨迹
     * @param traj  轨迹
     * @param timeSample  轨迹的采样频率
     * @param sim  是否是仿真，如果是仿真只会调用CobotStudio的机器人
     * @return  是否顺利执行完毕
     */
    bool executeTrajectory(rw::trajectory::StateTrajectory::Ptr traj, double timeSample = 0.1, bool sim = true);

    /**
     * 打开夹具
     * @return
     */
    int openHand();

    /**
     * 关闭夹具
     * @return
     */
    int closeHand();

    /**
     * 获取与CobotStudio交互的对象
     * @return
     */
    CobotStudioApi getCobotStudioApi();

    /**
     * 获取碰撞检测器对象
     * @return
     */
    rw::proximity::CollisionDetector::Ptr getCollisionDetector();

    /**
     * 获取State
     * @return
     */
    rw::kinematics::State getState();

    /**
     * 获取模型机器人对象
     * @return  模型机器人
     */
    rw::models::Device::Ptr getRobotDevice();

    /**
     * 火气3d相机的拍照对象
     * @return
     */
    std::shared_ptr<Falcon::Camera::Common::Camera3DClient> getCamera3DClient();

    /**
     * 获取机器人的驱动对象
     * @return
     */
    std::shared_ptr<Sensor::RobotDriver::RobotClient> getRobotClient();

    /**
     * 逆解 有可能回无法求解。该方法回姿态安全的关节值。
     * @param transform3D 笛卡尔
     * @return  返回安装的关节姿态
     */
    std::vector<rw::math::Q> solver(rw::math::Transform3D<> transform3D);

private:
    void initDevice();

private:
    /**
     * 场景文件路径
     */
    std::string _workCellPath;
    CobotStudioApi _cobotStudioApi;
    rw::models::WorkCell::Ptr _workCell;
    rw::proximity::CollisionDetector::Ptr _collisionDetector;
    rw::kinematics::State _state;
    rw::models::Device::Ptr _robotDevice;
    std::shared_ptr<Falcon::Camera::Common::Camera3DClient> _camera3DClient;
    std::shared_ptr<Sensor::RobotDriver::RobotClient> _robotClient;
    /**
     * 夹具modbus状态
     */
    modbus_t *_ctx;
};


#endif //ANTCOBOTSTUDIOAPI_ROBOTSCENE_H
