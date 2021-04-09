//
// Created by cobot on 2021/3/11.
//

#ifndef SPARROWC3TEST_DUALARMROBOTPLANNER_H
#define SPARROWC3TEST_DUALARMROBOTPLANNER_H

#include "rw/models.hpp"
#include "rw/models/WorkCell.hpp"
#include "rw/proximity.hpp"
#include "RWWorkCellParser.h"
#include "rw/trajectory/Trajectory.hpp"

/**
 * c3路径规划
 */
class DualArmRobotPlanner {
public:
    /**
     * 双臂规划器
     * @param workCell 场景
     * @param leftArm  左臂机器人名称
     * @param rightArm 右臂机器人名称
     */
    DualArmRobotPlanner(rw::models::WorkCell::Ptr workCell, std::string leftArm = "Left_Arm", std::string rightArm = "Right_Arm");

    /**
     * rrt规划路径
     * @param leftStart 左臂开始关节
     * @param rightStart 右臂开始关节
     * @param leftGoal 左臂结束关节
     * @param rightGoal 右边结束关节
     * @param state   开始状态
     * @param enableCons  是否启用限制
     * @return
     */
    rw::common::Ptr<rw::trajectory::Trajectory<rw::kinematics::State>>
    planQPath(rw::math::Q leftStart, rw::math::Q rightStart, rw::math::Q leftGoal, rw::math::Q rightGoal,
              rw::kinematics::State &state, boost::function<bool(rw::math::Q&)> stateChecker);

    /**
     * 左臂逆解获取一个没有碰撞的关节角
     * @param t 位姿
     * @param state 逆解的状态
     * @param tool 末端名称
     * @param isUR 是否是UR，可以选择封闭解
     * @return
     */
    std::vector<rw::math::Q>
    solverLeft(rw::math::Transform3D<> t, rw::kinematics::State &state, std::string tool, bool isUR = false);

    /**
     * 右臂逆解获取一个没有碰撞的关节角
     * @param t 位姿
     * @param state 逆解的状态
     * @return
     */
    std::vector<rw::math::Q>
    solverRight(rw::math::Transform3D<> t, rw::kinematics::State &state, std::string tool, bool isUR = false);

    /**
     * 选择合适的双臂关节角
     * @param leftQs 左臂的角度组合
     * @param rightQS  右臂的角度组合
     * @return   双臂不碰的解
     */
    std::vector<rw::math::Q>
    solver(std::vector<rw::math::Q> leftQs, std::vector<rw::math::Q> rightQS, rw::kinematics::State state);

    /**
     * 场景文件
     */
    rw::models::WorkCell::Ptr _workCell;
    /**
     * 碰撞检测
     */
    rw::proximity::CollisionDetector::Ptr _collisionDetector;
    /**
     * 左臂机器人运动学模型对象
     */
    rw::models::Device::Ptr _leftArmRobot;
    /**
     * 右臂机器人运动学模型对象
     */
    rw::models::Device::Ptr _rightArmRobot;
    /**
     * 双臂运动学机器人模型对象
     */
    rw::models::CompositeDevice::Ptr _c3CompositeRobot;
    /**
     * 机器人运动学状态
     */
    rw::kinematics::State _state;
    /**
     * 场景中左臂模型
     */
    std::string _leftArmName;
    /**
     * 场景中右臂模型
     */
    std::string _rightArmName;
private:
    rw::math::Q combineQ(rw::math::Q q1, rw::math::Q q2);


};


#endif //SPARROWC3TEST_DUALARMROBOTPLANNER_H
