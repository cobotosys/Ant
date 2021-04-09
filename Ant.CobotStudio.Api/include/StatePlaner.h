/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 2021-4-9          xuzhenhai
============================================================== **/

#pragma once

#include "boost/function.hpp"
#include "memory"
#include "rw/math.hpp"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "rw/trajectory.hpp"

/**
 * 路径规划目前使用的是ompl的路径规划
 */
class StatePlaner {
public:
    /**
     * 构造指定维度的路径规划器，维度和机器人的自由度绑定
     * @param dims  自由度
     */
    StatePlaner(int dims);

    /**
     * 状态检测，用来判断当前的采样关节值是否可以，可以根据关节是否碰撞，以及关节是否符合规划要求来判断。
     * 如果合理返回true
     * @param stateChecker
     */
    void setStateChecker(boost::function<bool(rw::math::Q &)> stateChecker);

    /**
     * 设置限制，每一个维度有包含一个最大值和最小值
     * @param bounds  限制
     */
    void setBounds(rw::math::Q bounds);

    rw::trajectory::QPath solve(double time, rw::math::Q start, rw::math::Q goal, double dis);

private:
    /**
     * 状态检测器
     */
    boost::function<bool(rw::math::Q &)> _stateChecker;
    std::shared_ptr<ompl::base::RealVectorStateSpace> _stateSpace;
    int _dims;
};