#include <logger/Logger.h>
#include "DualArmRobotScene.h"
#include "cobot/studio/common/CobotStudioApi.h"
#include "rw/kinematics/Kinematics.hpp"

void grasp(DualArmRobotScene &robot) {
    cobot::studio::common::CobotStudioApi cobotStudioApi;
    auto target = cobotStudioApi.getTran("Left_Arm.Base", "LeftPose");//LeftPose为视觉给的抓取点，注意Y方向是长边方向
    auto leftTarget = target * rw::math::Transform3D<>({0, 0.07, 0});//沿着y正方向增加 y对应夹具的缺口
    auto rightTarget = target * rw::math::Transform3D<>({0, -0.07, 0});//沿着负正方向增加 y对应夹具的缺口
    if (leftTarget.P().norm2() > rightTarget.P().norm2()) { //两个姿态离左侧机器人近就由左侧机器人抓取，否则使用默认的抓取吮吸
        auto temp = leftTarget;
        leftTarget = rightTarget;
        rightTarget = temp;
    }
    //下两行计算右臂抓取点
    auto rightToLeftBase = cobotStudioApi.getTran("Right_Arm.Base", "Left_Arm.Base");
    rightTarget = rightToLeftBase * rightTarget;
    //设置到场景用于测试是否计算正常
    cobotStudioApi.setAxes({leftTarget}, "Left_Arm.Base");
    cobotStudioApi.setAxes({rightTarget}, "Right_Arm.Base");
    //双臂协作运动到抓取点
    robot.moveToFreely(leftTarget, rightTarget);
}

void move(DualArmRobotScene &robot, rw::math::Transform3D<> t, rw::math::Q leftQ, rw::math::Q rightQ) {
    robot._planner->_leftArmRobot->setQ(leftQ, robot._planner->_state);
    robot._planner->_rightArmRobot->setQ(rightQ, robot._planner->_state);
    auto origT = rw::kinematics::Kinematics::frameTframe(robot._planner->_leftArmRobot->getEnd(),
                                                         robot._planner->_rightArmRobot->getEnd(),
                                                         robot._planner->_state);
    cobot::studio::common::CobotStudioApi cobotStudioApi;
    auto target = cobotStudioApi.getTran("Left_Arm.Base", "LeftPose");//LeftPose为视觉给的抓取点，注意Y方向是长边方向
    target = target * t;
    auto leftTarget = target * rw::math::Transform3D<>({0, 0.07, 0});//沿着y正方向增加 y对应夹具的缺口
    auto rightTarget = target * rw::math::Transform3D<>({0, -0.07, 0});//沿着负正方向增加 y对应夹具的缺口
    if (leftTarget.P().norm2() > rightTarget.P().norm2()) { //两个姿态离左侧机器人近就由左侧机器人抓取，否则使用默认的抓取吮吸
        auto temp = leftTarget;
        leftTarget = rightTarget;
        rightTarget = temp;
    }
    //下两行计算右臂抓取点
    auto rightToLeftBase = cobotStudioApi.getTran("Right_Arm.Base", "Left_Arm.Base");
    rightTarget = rightToLeftBase * rightTarget;
    //设置到场景用于测试是否计算正常
    cobotStudioApi.setAxes({leftTarget}, "Left_Arm.Base");
    cobotStudioApi.setAxes({rightTarget}, "Right_Arm.Base");
    //双臂协作运动到抓取点
    auto stateCheck = [&, origT](rw::math::Q &q) {
        robot._planner->_c3CompositeRobot->setQ(q, robot._planner->_state);
        bool b = robot._planner->_collisionDetector->inCollision(robot._planner->_state);
        if (b) {
            return false;
        } else {
            auto newT = rw::kinematics::Kinematics::frameTframe(robot._planner->_rightArmRobot->getEnd(),
                                                                robot._planner->_leftArmRobot->getEnd(),
                                                                robot._planner->_state) * origT;
            return newT.equal(rw::math::Transform3D<>::identity(), 0.04);
        }
//        return true;
    };
    robot.moveToConstraint(leftTarget, rightTarget, stateCheck);
}

int main() {
    DualArmRobotScene robot;
    grasp(robot);
    cobot::studio::common::CobotStudioApi cobotStudioApi;
    auto leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
    auto rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
    move(robot,rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
//    move(rw::math::Transform3D<>({0.08, 0.01, -0.05}), leftQ, rightQ);

    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
    move(robot,rw::math::Transform3D<>({0.04, 0, -0.06}), leftQ, rightQ);


    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
    move(robot,rw::math::Transform3D<>({0.08, 0, -0.06}), leftQ, rightQ);

    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
    move(robot,rw::math::Transform3D<>({0.08, 0.02, -0.06}), leftQ, rightQ);

    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
    move(robot,rw::math::Transform3D<>({0.08, -0.02, -0.06}), leftQ, rightQ);

    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
    move(robot,rw::math::Transform3D<>({0, 0, 0}), leftQ, rightQ);
//
//
//    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
//    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
////    move(rw::math::Transform3D<>({0, 0, -0.06}), leftQ, rightQ);
//    move(rw::math::Transform3D<>({-0.04, 0, 0}), leftQ, rightQ);

//    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
//    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0.1, 0, 0}), leftQ, rightQ);
//
//    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
//    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({-0.1, 0, 0}), leftQ, rightQ);


//    leftQ = rw::math::Q(cobotStudioApi.getQ("Left_Arm"));
//    rightQ = rw::math::Q(cobotStudioApi.getQ("Right_Arm"));
//    move(rw::math::Transform3D<>({0, 0, 0.1}), leftQ, rightQ);
//    LOG_INFO << leftQ;
//    LOG_INFO << rightQ;
}